#!/usr/bin/env python3
# coding: utf-8

import rclpy
from rclpy.node import Node
from enum import Enum, auto
import time

from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Trigger
from std_msgs.msg import String, Int32

from qulido_robot_msgs.action import ExecuteMotion
from qulido_robot_msgs.msg import MotionPrimitive, MotionSequence
from qulido_robot_msgs.srv import GetBoardState
from qulido_robot_msgs.srv import AiCompute

BOARD_X_MIN = 280
BOARD_X_MAX = 674
BOARD_Y_MIN = -185
BOARD_Y_MAX = 210
pick_pose = [267.358, 7.489, 194.298, 131.655, 179.966, -138.5]

# ===================== FSM STATES =====================
class OrchestratorState(Enum):
    WAIT_WAKE = auto()
    WAIT_START = auto()
    HUMAN_TURN = auto()
    ROBOT_THINK = auto()
    ROBOT_PLAN = auto()
    ROBOT_EXECUTE = auto()
    CLEAN_UP = auto()
    ERROR = auto()


# ===================== NODE =====================
class GameOrchestratorNode(Node):

    def __init__(self):
        super().__init__('game_orchestrator')

        # ---------- Parameters ----------
        self.declare_parameter("wake_service", "/wakeup_robot")
        self.declare_parameter("speech_service", "/speech_to_text")
        self.declare_parameter("vision_service", "/vision/get_board_state")
        self.declare_parameter("ai_service", "/ai_agent/get_robot_move")
        self.declare_parameter("motion_action", "/execute_motion")

        self.wake_srv = self.get_parameter("wake_service").value
        self.speech_srv = self.get_parameter("speech_service").value
        self.vision_srv = self.get_parameter("vision_service").value
        self.ai_srv = self.get_parameter("ai_service").value
        self.motion_action_name = self.get_parameter("motion_action").value

        # ---------- Pub ----------
        self.difficulty_pub = self.create_publisher(Int32, "/game_level", 10)
        self.state_pub = self.create_publisher(String, '/now_game_state', 10)

        # ---------- Sub ----------

        self.game_finished_sub = self.create_subscription(
            String,
            "/game_finished",
            self.game_finished_callback,
            10
        )

        # ---------- Clients ----------
        self.wake_client = self.create_client(Trigger, self.wake_srv)
        self.speech_client = self.create_client(Trigger, self.speech_srv)
        self.vision_client = self.create_client(GetBoardState, self.vision_srv)
        self.ai_client = self.create_client(AiCompute, self.ai_srv)

        self.motion_client = ActionClient(self, ExecuteMotion, self.motion_action_name)

        # ---------- FSM ----------
        self.state = OrchestratorState.WAIT_WAKE

        # ---------- Futures ----------
        self.wake_future = None
        self.speech_future = None
        self.vision_future = None
        self.ai_future = None

        self.motion_goal_future = None
        self.motion_result_future = None

        # ---------- Game Data ----------
        self.game_state = [[1, 6, 3], [-1, 0, 3]]
        self.prev_state = []
        self.added = []
        self.robot_cmd = None
        self.robot_motion = None
        self.game_winner = None
        self._human_turn_initialized = False  # Human Turn 진입 시 메시지/vision 호출용
        self._awaiting_final_state = False    # end turn 후 vision 대기용
        self.wall_used=0

        self.camera_pose = [160.018, 7.227, 347.286, 0.043, 150.865, 89.941]
        self.pose_home = [367.438, 7.224, 194.564, 68.625, 179.97, 68.493]
        self.wall_pose = [
            [257.072, -137.373, 54.029, 147.491, 179.923, -122.683],
            [257.066, -77.279, 54.043, 146.731, 179.926, -123.454],
            [256.932, -16.599, 53.945, 151.618, 179.911, -118.519],
            [256.998, 42.637, 53.959, 150.961, 179.912, -119.197],
            [257.055, 102.752, 53.982, 149.117, 179.914, -121.052],
            [257.449, 160.218, 54.289, 114.474, 179.924, -155.735]
        ]

        self.create_timer(0.1, self.main_loop)
        self.log("✅ Orchestrator initialized (Future FSM)")

    # ==================================================
    def log(self, text):
        msg = String()
        msg.data = text
        self.get_logger().info(text)

    # ==================================================
    def main_loop(self):

        try:
            # ---------------- WAIT_WAKE ----------------
            if self.state == OrchestratorState.WAIT_WAKE:
                msg = String()
                msg.data = self.state.name
                self.state_pub.publish(msg)

                if self.wake_future is None:
                    self.log("Say '헤이 쿼리' to begin.")
                    if self.wake_client.wait_for_service(timeout_sec=0.0):
                        self.wake_future = self.wake_client.call_async(Trigger.Request())
                        self.log("Wakeup listening...")
                    return

                if self.wake_future.done():
                    res = self.wake_future.result()
                    self.wake_future = None

                    if res and res.success and "awake" in res.message.lower():
                        self.log("Robot awakened → WAIT_START")
                        self.state = OrchestratorState.WAIT_START

            # ---------------- WAIT_START ----------------
            elif self.state == OrchestratorState.WAIT_START:
                msg = String()
                msg.data = self.state.name
                self.state_pub.publish(msg)

                if self.speech_future is None:
                    self.log("Say 'start game' to begin.")
                    if self.speech_client.wait_for_service(timeout_sec=0.0):
                        self.speech_future = self.speech_client.call_async(Trigger.Request())
                    return

                if self.speech_future.done():
                    res = self.speech_future.result()
                    self.speech_future = None

                    if res and "start game" in res.message.lower():
                        try:
                            _, difficulty = res.message.lower().split(',', 1)
                            self.publish_difficulty(difficulty)
                        except ValueError:
                            self.get_logger().warn(
                                f"Failed to parse difficulty from: {res.message}"
                            )

                        self.log("Game started → HUMAN_TURN")
                        self.state = OrchestratorState.HUMAN_TURN


            # ---------------- HUMAN_TURN ----------------
# ---------------- HUMAN_TURN ----------------
            elif self.state == OrchestratorState.HUMAN_TURN:
                msg = String()
                msg.data = self.state.name
                self.state_pub.publish(msg)
                
                # --- Step 1: Human Turn 진입 시 한 번 실행 ---
                if not self._human_turn_initialized:
                    self._human_turn_initialized = True
                    self._human_turn_started = False 
                    self.log("Human's turn → moving to camera pose first.")

                    # 로봇을 camera_pose로 이동시키기 위한 motion 생성
                    motion = {
                        'sequence': [
                            {'primitive': 'move_pose', 'pose': self.camera_pose}
                        ]
                    }
                    goal = self.build_motion_goal(motion)
                    self.motion_goal_future = self.motion_client.send_goal_async(goal)
                    return  # motion이 완료될 때까지 대기

                # --- Step 2: Goal 완료 확인 및 Result Future 생성 ---
                if self.motion_goal_future and self.motion_goal_future.done():
                    goal_handle = self.motion_goal_future.result()  # ClientGoalHandle
                    self.motion_goal_future = None

                    if not goal_handle.accepted:
                        self.log("Motion goal rejected → ERROR")
                        self.state = OrchestratorState.ERROR
                        return

                    self.motion_result_future = goal_handle.get_result_async()
                    return

                # --- Step 3: Motion 결과 처리 ---
                if self.motion_result_future and self.motion_result_future.done():
                    result = self.motion_result_future.result().result
                    self.motion_result_future = None

                    if result.success:
                        self.log("Arrived at camera pose → requesting board state")
                        # Vision 호출
                        if self.vision_client.wait_for_service(timeout_sec=0.0):
                            self.vision_future = self.vision_client.call_async(GetBoardState.Request())
                    else:
                        self.log("Motion failed → ERROR")
                        self.state = OrchestratorState.ERROR
                    return

                # --- Step 4: Vision 결과 처리 (Human Turn 시작 시) ---
                if self.vision_future and self.vision_future.done() and not self._human_turn_started:
                    res = self.vision_future.result()
                    self.vision_future = None

                    self.game_state = [row.data for row in res.board_state]
                    self.prev_state = self.game_state.copy()  # 기준 상태 저장
                    self._human_turn_started = True
                    self.log(f"Human Turn started, prev_state saved: {self.prev_state}")

                # --- Step 5: Speech 감지 ---
                if not self._awaiting_final_state and self._human_turn_started:
                    if self.speech_future is None:
                        if self.speech_client.wait_for_service(timeout_sec=0.0):
                            self.speech_future = self.speech_client.call_async(Trigger.Request())

                    elif self.speech_future.done():
                        res = self.speech_future.result()
                        self.speech_future = None

                        if "end turn" in res.message.lower():
                            self.log("Human ended turn → fetching final game state")
                            # Vision 다시 호출
                            if self.vision_client.wait_for_service(timeout_sec=0.0):
                                self.vision_future = self.vision_client.call_async(GetBoardState.Request())
                            self._awaiting_final_state = True
                            return

                # --- Step 6: End turn 후 Vision 결과 처리 ---
                if self._awaiting_final_state:
                    if self.vision_future and self.vision_future.done():
                        res = self.vision_future.result()
                        self.vision_future = None

                        current_state = [row.data for row in res.board_state]

                        # prev_state와 비교 → self.added 계산
                        self.added = self.compute_added(self.prev_state, current_state)
                        self.log(f"Detected human additions/moves: {self.added}")

                        # 상태 초기화 및 다음 FSM으로 전환
                        self._human_turn_initialized = False
                        self._awaiting_final_state = False
                        self.state = OrchestratorState.ROBOT_THINK



            # ---------------- ROBOT_THINK ----------------
            elif self.state == OrchestratorState.ROBOT_THINK:
                msg = String()
                msg.data = self.state.name
                self.state_pub.publish(msg)

                if self.ai_future is None:
                    if self.ai_client.wait_for_service(timeout_sec=0.0):
                        req = AiCompute.Request()
                        req.added = self.added
                        self.ai_future = self.ai_client.call_async(req)
                    return

                if self.ai_future.done():
                    res = self.ai_future.result()
                    self.ai_future = None
                    self.robot_cmd = res.ai_cmd
                    self.state = OrchestratorState.ROBOT_PLAN

            # ---------------- ROBOT_PLAN ----------------
            elif self.state == OrchestratorState.ROBOT_PLAN:
                msg = String()
                msg.data = self.state.name
                self.state_pub.publish(msg)

                self.robot_motion = self.plan_motion(self.robot_cmd)
                self.state = OrchestratorState.ROBOT_EXECUTE

            # ---------------- ROBOT_EXECUTE ----------------
            elif self.state == OrchestratorState.ROBOT_EXECUTE:
                msg = String()
                msg.data = self.state.name
                self.state_pub.publish(msg)

                if self.motion_goal_future is None:
                    goal = self.build_motion_goal(self.robot_motion)
                    self.motion_goal_future = self.motion_client.send_goal_async(goal)
                    return

                if self.motion_goal_future.done() and self.motion_result_future is None:
                    goal_handle = self.motion_goal_future.result()
                    if not goal_handle.accepted:
                        self.state = OrchestratorState.ERROR
                        return
                    self.motion_result_future = goal_handle.get_result_async()
                    return

                if self.motion_result_future and self.motion_result_future.done():
                    result = self.motion_result_future.result().result
                    self.motion_goal_future = None
                    self.motion_result_future = None

                    if result.success:
                        self.log("Robot move done → HUMAN_TURN")
                        self.state = OrchestratorState.HUMAN_TURN
                    else:
                        self.state = OrchestratorState.ERROR

            # ---------------- CLEAN_UP ----------------
            elif self.state == OrchestratorState.CLEAN_UP:
                pass
            # ---------------- ERROR ----------------
            elif self.state == OrchestratorState.ERROR:
                self.log("ERROR → resetting to WAIT_WAKE")
                self.state = OrchestratorState.WAIT_WAKE

        except Exception as e:
            self.log(f"FSM exception: {e}")
            self.state = OrchestratorState.ERROR

    # ==================================================

    def game_finished_callback(self, msg: String):
        """
        /game_finished 토픽에서 AI 혹은 player 정보를 받아서
        self.game_winner에 저장하고 CLEAN_UP 상태로 전환
        """
        winner = msg.data.strip().lower()
        if winner in ["ai", "player"]:
            self.game_winner = winner
            self.log(f"Game finished → winner: {winner}")
            self.state = OrchestratorState.CLEAN_UP
        else:
            self.get_logger().warn(f"Unknown winner message: {msg.data}")


    def publish_difficulty(self, difficulty_str: str):
        difficulty_map = {
            "easy": 0,
            "normal": 1,
            "hard": 2
        }

        difficulty_str = difficulty_str.strip().lower()

        if difficulty_str not in difficulty_map:
            self.get_logger().warn(f"Unknown difficulty: {difficulty_str}")
            return

        msg = Int32()
        msg.data = difficulty_map[difficulty_str]
        self.difficulty_pub.publish(msg)

        self.get_logger().info(
            f"🎮 Game difficulty set: {difficulty_str.upper()} ({msg.data})"
        )

    # ==================================================
    def compute_added(self, prev, current):
        """
        prev_state와 current_state를 비교해서 새로 추가된 요소만 반환
        prev: 이전 상태 [[...], [...], ...]
        current: 현재 상태 [[...], [...], ...]
        반환: 추가된 요소 [[x, y, z]] 형태 (리스트 안에 원소 하나)
        """

        # prev의 각 원소를 set으로 만들어서 비교
        prev_set = set(tuple(x) for x in prev)
        current_set = set(tuple(x) for x in current)

        # current에는 있는데 prev에는 없는 원소 → 새로 추가된 것
        added = list(current_set - prev_set)

        # added는 리스트 안에 한 원소만 넣기
        return list(added[0]) if added else []

    # ==================================================
    def build_motion_goal(self, motion):
        goal = ExecuteMotion.Goal()
        goal.sequence = MotionSequence()

        for step in motion["sequence"]:
            prim = MotionPrimitive()
            prim.primitive = step["primitive"]
            prim.target_pose = step.get("pose", [0.0]*6)
            prim.gripper_width = step.get("width", 0)
            goal.sequence.sequence.append(prim)

        return goal

    # ==================================================
    def plan_motion(self, cmd):
        obj = cmd[0]
        pos_b = cmd[1:]

        print(cmd)

        if obj==-1: # AI pawn
            for state in self.game_state:
                if state[0] == -1:
                    pos_obj_b = state[1:]
                    break
            pos = self.pawn_board_to_base(*pos_b)
            pos_obj = self.pawn_board_to_base(*pos_obj_b)
            pos_pre = pos.copy()
            pos_pre[2] += 40
            pos_obj_pre = pos_obj.copy()
            pos_obj_pre[2] += 40
            motion = {
                'sequence': [
                    {'primitive': 'operate_gripper', 'width': 350},
                    {'primitive': 'move_pose', 'pose': pick_pose},
                    {'primitive': 'move_pose', 'pose': pos_obj_pre},
                    {'primitive': 'move_pose', 'pose': pos_obj},
                    {'primitive': 'operate_gripper', 'width': 0},
                    {'primitive': 'move_pose', 'pose': pos_obj_pre},
                    {'primitive': 'move_pose', 'pose': pos_pre},
                    {'primitive': 'move_pose', 'pose': pos},
                    {'primitive': 'force_control'},
                    {'primitive': 'operate_gripper', 'width': 350},
                    {'primitive': 'move_pose', 'pose': pos_pre},
                    {'primitive': 'move_pose', 'pose': pick_pose}
                ]
            }
        else: # wall
            pos_obj = self.wall_pose[self.wall_used]
            self.wall_used+=1
            if obj==-2:
                pos = self.wall_board_to_base(*pos_b, "horizontal")
            elif obj==2:
                pos = self.wall_board_to_base(*pos_b, "vertical")
            pos_pre = pos.copy()
            pos_pre[2] += 40
            pos_obj_pre = pos_obj.copy()
            pos_obj_pre[2] += 40
            motion = {
                'sequence': [
                    {'primitive': 'operate_gripper', 'width': 250},
                    {'primitive': 'move_pose', 'pose': pick_pose},
                    {'primitive': 'move_pose', 'pose': pos_obj_pre},
                    {'primitive': 'move_pose', 'pose': pos_obj},
                    {'primitive': 'operate_gripper', 'width': 0},
                    {'primitive': 'move_pose', 'pose': pos_obj_pre},
                    {'primitive': 'move_pose', 'pose': pos_pre},
                    {'primitive': 'move_pose', 'pose': pos},
                    {'primitive': 'force_control'},
                    {'primitive': 'operate_gripper', 'width': 250},
                    {'primitive': 'move_pose', 'pose': pos_pre},
                    {'primitive': 'move_pose', 'pose': pick_pose}
                ]
            }
        print(motion)

        return motion

    def pawn_board_to_base(self, r, c):
        """
        pawn 보드좌표 (r,c) → base 좌표계 pose
        """
        cell_w = (BOARD_X_MAX - BOARD_X_MIN) / 7
        cell_h = (BOARD_Y_MAX - BOARD_Y_MIN) / 7

        x = float(BOARD_X_MIN + (r + 0.5) * cell_w)
        y = float(BOARD_Y_MIN + (c + 0.5) * cell_h)
        z = float(30)
        rx, ry, rz = map(float, pick_pose[3:])
        return [x, y, z, rx, ry, rz]

    def wall_board_to_base(self, r, c, orientation):
        """
        wall 보드좌표 (r,c) → base 좌표
        orientation: "horizontal" | "vertical"
        """
        pawn_cell_w = (BOARD_X_MAX - BOARD_X_MIN) / 7
        pawn_cell_h = (BOARD_Y_MAX - BOARD_Y_MIN) / 7

        wall_x_min = BOARD_X_MIN + pawn_cell_w / 2
        wall_x_max = BOARD_X_MAX - pawn_cell_w / 2
        wall_y_min = BOARD_Y_MIN + pawn_cell_h / 2
        wall_y_max = BOARD_Y_MAX - pawn_cell_h / 2

        wall_cell_w = (wall_x_max - wall_x_min) / 6
        wall_cell_h = (wall_y_max - wall_y_min) / 6

        x = wall_x_min + (r + 0.5) * wall_cell_w
        y = wall_y_min + (c + 0.5) * wall_cell_h

        z = 30
        rx, ry, rz = pick_pose[3:]

        if orientation == "horizontal":
            rz += 90
            y += 25
        else:
            x += 25 # 잡는위치 보정

        return list(map(float, [x, y, z, rx, ry, rz]))


# ===================== MAIN =====================
def main(args=None):
    rclpy.init(args=args)
    node = GameOrchestratorNode()
    executor = MultiThreadedExecutor(4)

    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()