#!/usr/bin/env python3
# coding: utf-8

"""
FULL ORCHESTRATOR (UPDATED)
- Human turn: continuous vision + speech "end turn"
- Robot turn: AI -> plan -> motion -> move to look_pose -> vision verify
"""

import rclpy
from rclpy.node import Node
from enum import Enum, auto
import time
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Trigger
from std_msgs.msg import String
from qulido_robot_msgs.action import ExecuteMotion
from qulido_robot_msgs.msg import MotionPrimitive, MotionSequence
from qulido_robot_msgs.srv import GetBoardState
from qulido_robot_msgs.srv import AiCompute


class OrchestratorState(Enum):
    WAIT_WAKE = auto()
    WAIT_START = auto()
    HUMAN_TURN = auto()
    ROBOT_THINK = auto()
    ROBOT_PLAN = auto()
    ROBOT_EXECUTE = auto()
    ROBOT_VISION_VERIFY = auto()
    ERROR = auto()


class GameOrchestratorNode(Node):

    def __init__(self):
        super().__init__('game_orchestrator')

        # Parameters
        self.declare_parameter("speech_service", "/speech_to_text")
        self.declare_parameter("wake_service", "/wakeup_robot")
        self.declare_parameter("vision_service", "/vision/get_board_state")
        self.declare_parameter("ai_service", "/ai_agent/get_robot_move")
        self.declare_parameter("motion_action", "/execute_motion")
        self.declare_parameter("game_state_topic", "/qulido/game_state")

        self.speech_srv = self.get_parameter("speech_service").value
        self.wake_srv = self.get_parameter("wake_service").value
        self.vision_srv = self.get_parameter("vision_service").value
        self.ai_srv = self.get_parameter("ai_service").value
        self.motion_action_name = self.get_parameter("motion_action").value
        self.game_state_topic = self.get_parameter("game_state_topic").value

        # Pub
        self.status_pub = self.create_publisher(String, "/qulido/orch_status", 10)
        self.game_state_pub = self.create_publisher(String, self.game_state_topic, 10)

        # Service Clients
        self.speech_client = self.create_client(Trigger, self.speech_srv)
        self.wake_client = self.create_client(Trigger, self.wake_srv)
        self.vision_client = self.create_client(GetBoardState, self.vision_srv)
        self.ai_client = self.create_client(AiCompute, self.ai_srv)

        # Motion Action
        self.motion_client = ActionClient(self, ExecuteMotion, self.motion_action_name)

        # Internal
        self.state = OrchestratorState.WAIT_START
        self.game_state = []
        self.wall_used = 0 # 사용한 로봇의 벽 개수
        self.wall_pose = [[]] # 로봇 벽 초기 위치
        self.prev_state = []
        self.added = []

        self.create_timer(0.2, self.main_loop)

        self.log("Orchestrator initialized.")

    # ------------------------------------------------------------------
    def log(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def wait_service(self, client, timeout=3.0):
        start = time.time()
        while not client.wait_for_service(timeout_sec=1.0):
            if time.time() - start > timeout:
                return False
        return True
    
    # ---------------- WAKE -----------------
    def call_wake(self, timeout=3.0):
        if not self.wait_service(self.wake_client, 2.0):
            return False, ""
        req = Trigger.Request()
        fut = self.wake_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            return False, ""
        return True, fut.result().message

    # ---------------- SPEECH -----------------
    def call_speech(self, timeout=3.0):
        if not self.wait_service(self.speech_client, 2.0):
            return False, ""
        req = Trigger.Request()
        fut = self.speech_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            return False, ""
        return True, fut.result().message

    # ---------------- VISION -----------------
    def call_vision(self, timeout=3.0):
        if not self.wait_service(self.vision_client, 2.0):
            return False, None
        req = GetBoardState.Request()
        fut = self.vision_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            return False, None
        
        res = fut.result()

        try:
            board = [row.data for row in res.board_state]
            return True, board
        except:
            return False, None

    # ---------------- AI -----------------
    def call_ai(self, board, timeout=5.0):
        if not self.wait_service(self.ai_client, 3.0):
            return False, None
        req = AiCompute.Request()
        req.added = self.added
        fut = self.ai_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            return False, None

        res = fut.result()
        ai_cmd = res.ai_cmd
        return True, ai_cmd

    # ---------------- PUBLISH -----------------
    def publish_game_state(self):
        msg = String()
        msg.data = str(self.game_state)
        self.game_state_pub.publish(msg)

    # ------------------------------------------------------------------
    # MOTION + GEOMETRY
    # ------------------------------------------------------------------

    def plan_motion(self, cmd):
        obj = cmd[0]
        pos = cmd[1:]

        if obj==-1: # AI pawn
            for state in self.game_state:
                if state[0] == -1:
                    pos_obj = state[1:]
            #pos, pos_obj 변환
            pos_pre = pos + [0,0,40,0,0,0]
            pos_obj_pre = pos_obj + [0,0,40,0,0,0]
            motion = {
                'sequence': [
                    {'primitive': 'operate_gripper', 'width': 300},
                    {'primitive': 'move_pose', 'pose': pos_obj_pre},
                    {'primitive': 'move_pose', 'pose': pos_obj},
                    {'primitive': 'operate_gripper', 'width': 0},
                    {'primitive': 'move_pose', 'pose': pos_obj_pre},
                    {'primitive': 'move_pose', 'pose': pos_pre},
                    {'primitive': 'force_control'},
                    {'primitive': 'operate_gripper', 'width': 300},
                    {'primitive': 'move_pose', 'pose': pos_pre},
                ]
            }
        elif obj==2: # vertical wall
            pos_obj = self.wall_pose[self.wall_used]
            #pos, pos_obj 변환
            pos_pre = pos + [0,0,40,0,0,0]
            pos_obj_pre = pos_obj + [0,0,40,0,0,0]
            motion = {
                'sequence': [
                    {'primitive': 'operate_gripper', 'width': 200},
                    {'primitive': 'move_pose', 'pose': pos_obj_pre},
                    {'primitive': 'move_pose', 'pose': pos_obj},
                    {'primitive': 'operate_gripper', 'width': 0},
                    {'primitive': 'move_pose', 'pose': pos_obj_pre},
                    {'primitive': 'move_pose', 'pose': pos_pre},
                    {'primitive': 'force_control'},
                    {'primitive': 'operate_gripper', 'width': 200},
                    {'primitive': 'move_pose', 'pose': pos_pre},
                ]
            }
        elif obj==-2: # horizontal wall
            pos_obj = self.wall_pose[self.wall_used]
            #pos, pos_obj 변환
            pos_pre = pos + [0,0,40,0,0,0] + [0,0,0,0,0,90]
            pos_obj_pre = pos_obj + [0,0,40,0,0,0]
            motion = {
                'sequence': [
                    {'primitive': 'operate_gripper', 'width': 200},
                    {'primitive': 'move_pose', 'pose': pos_obj_pre},
                    {'primitive': 'move_pose', 'pose': pos_obj},
                    {'primitive': 'operate_gripper', 'width': 0},
                    {'primitive': 'move_pose', 'pose': pos_obj_pre},
                    {'primitive': 'move_pose', 'pose': pos_pre},
                    {'primitive': 'force_control'},
                    {'primitive': 'operate_gripper', 'width': 200},
                    {'primitive': 'move_pose', 'pose': pos_pre},
                ]
            }

        return True, motion

    def execute_motion(self, motion):

        if not self.motion_client.wait_for_server(timeout_sec=3.0):
            self.log("Motion server not ready.")
            return False

        # -----------------------------
        # 1) Build goal message
        # -----------------------------
        goal_msg = ExecuteMotion.Goal()
        goal_msg.sequence = MotionSequence()

        for step in motion["sequence"]:
            prim = MotionPrimitive()
            prim.primitive = step["primitive"]

            # move_pose
            if step["primitive"] == "move_pose":
                prim.target_pose = step["pose"]  # list[6]
                prim.gripper_width = 0  # unused

            # operate_gripper
            elif step["primitive"] == "operate_gripper":
                prim.gripper_width = step["width"]  # int
                prim.target_pose = [0.0]*6  # unused, must be 6 floats

            # force_control
            elif step["primitive"] == "force_control":
                prim.target_pose = [0.0]*6
                prim.gripper_width = 0

            goal_msg.sequence.sequence.append(prim)

        # -----------------------------
        # 2) Send action goal
        # -----------------------------
        future = self.motion_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        if not future.done():
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.log("Motion goal rejected.")
            return False

        # -----------------------------
        # 3) Wait for result
        # -----------------------------
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        if not result_future.done():
            return False

        result = result_future.result().result
        return result.success


    # ------------------------------------------------------------------
    # FSM
    # ------------------------------------------------------------------
    def main_loop(self):
        try:
            
            # ---------------- WAIT_WAKE ----------------
            if self.state == OrchestratorState.WAIT_WAKE:
                self.log("Say '헤이 쿼리' to begin.")
                ok, txt = self.call_wake()
                if ok and 'awake' in txt.lower():
                    self.log("Robot awaken → WAIT_START")
                    self.state = OrchestratorState.WAIT_START

            # ---------------- WAIT_START ----------------
            elif self.state == OrchestratorState.WAIT_START:
                self.log("Say 'start game' to begin.")
                ok, txt = self.call_speech()
                if ok and 'start' in txt.lower():
                    self.log("Game started → HUMAN_TURN")
                    self.state = OrchestratorState.HUMAN_TURN

            # ---------------- HUMAN_TURN -----------------
            elif self.state == OrchestratorState.HUMAN_TURN:

                # Move to look pose first
                self.log("Moving to look pose for human turn.")
                self.execute_motion({
                    'sequence':[{'primitive':'move_pose','pose':[0, 0, 0, 0, 0, 0]}]
                })

                self.log("Observing human moves... say 'end turn' to finish.")

                self.prev_state = self.game_state

                while rclpy.ok():

                    # 1) Vision update
                    vok, board = self.call_vision(timeout=1.0)
                    if vok:
                        self.game_state = board
                        self.publish_game_state()

                    # 2) Speech check
                    sok, stxt = self.call_speech(timeout=0.15)
                    if sok and ("end" in stxt.lower() or "done" in stxt.lower()):
                        self.log("Human ended turn.")
                        prev_set = {tuple(x) for x in self.prev_state}
                        curr_set = {tuple(x) for x in self.game_state}
                        self.added = list(curr_set - prev_set)
                        self.state = OrchestratorState.ROBOT_THINK
                        break

                    time.sleep(0.3)

            # ---------------- ROBOT_THINK -----------------
            elif self.state == OrchestratorState.ROBOT_THINK:
                ok, cmd = self.call_ai(self.added)
                if not ok:
                    self.state = OrchestratorState.ERROR
                else:
                    self.robot_cmd = cmd
                    self.state = OrchestratorState.ROBOT_PLAN

            # ---------------- ROBOT_PLAN ------------------
            elif self.state == OrchestratorState.ROBOT_PLAN:
                ok, motion = self.plan_motion(self.robot_cmd)
                if not ok:
                    self.state = OrchestratorState.ERROR
                else:
                    self.robot_motion = motion
                    self.state = OrchestratorState.ROBOT_EXECUTE

            # ---------------- ROBOT_EXECUTE ----------------
            elif self.state == OrchestratorState.ROBOT_EXECUTE:
                ok = self.execute_motion(self.robot_motion)
                if not ok:
                    self.state = OrchestratorState.ERROR
                else:
                    self.state = OrchestratorState.HUMAN_TURN

            # ---------------- ROBOT_VISION_VERIFY ---------
            # elif self.state == OrchestratorState.ROBOT_VISION_VERIFY:

            #     # *** NEW: Always move to look_pose before vision ***
            #     self.log("Robot move complete → moving to look_pose for verification.")
            #     self.execute_motion({
            #         'sequence':[{'primitive':'move_pose','pose':[0, 0, 0, 0, 0, 0]}]
            #     })

            #     self.log("Performing vision verification...")

            #     ok, board = self.call_vision()
            #     if ok:
            #         self.game_state = board
            #         self.publish_game_state()
            #         self.log("Robot move verified → HUMAN_TURN")
            #         self.state = OrchestratorState.HUMAN_TURN
            #     else:
            #         self.log("Vision verify failed.")
            #         self.state = OrchestratorState.ERROR

            # ---------------- ERROR ------------------------
            elif self.state == OrchestratorState.ERROR:
                self.log("Error occurred. Resetting to HUMAN_TURN.")
                self.state = OrchestratorState.HUMAN_TURN

        except Exception as e:
            self.log(f"Exception in main_loop: {e}")
            self.state = OrchestratorState.ERROR


# ------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = GameOrchestratorNode()
    executor = MultiThreadedExecutor(4)

    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        node.log("Shutting down orchestrator.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
