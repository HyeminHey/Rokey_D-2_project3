#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String, Bool
from rclpy.action import ActionClient

from qulido_robot_msgs.srv import GetBoardState
from qulido_robot_msgs.msg import MotionPrimitive, MotionSequence
from qulido_robot_msgs.action import ExecuteMotion
from qulido_robot_msgs.srv import CleanUpTrigger



pick_pose = [0.004, -15.49, 103.192, 0.041, 92.317, 90.012] # joint
pick_pose_l = [257.256, 7.46, 194.059, 159.331, 179.928, -110.819]


class CleanUpNode(Node):

    def __init__(self):
        super().__init__('clean_up_node')

        # ---------- Service Server ----------
        self.srv = self.create_service(
            CleanUpTrigger,
            '/clean_up',
            self.on_cleanup_request
        )

        # ---------- Clients ----------
        self.vision_client = self.create_client(GetBoardState, "/vision/get_board_state")
        self.motion_client = ActionClient(self, ExecuteMotion, "/execute_motion")

        # -----------Publisher---------
        self.end_pub = self.create_publisher(Bool, '/clean_up_finished', 10)

        # ---------- State ----------
        self._active = False
        self._response = None

        # ---------- Cleanup Data ----------
        self.detected_pawns = []
        self.detected_walls = []
        self.seq_list = []

        self.current_phase = None   # "PAWN" | "WALL"
        self.current_idx = 0
        self.sorted_walls = 0
        self.wall_used = 0

        # ---------- Poses (TODO: 실제 값) ----------
        self.detection_pose = None
        self.pawn_origin_pose = None
        self.wall_origin_pose = None
        self.camera_pose = [-29.062, -49.05, 96.263, 13.934, 106.247, 68.124] # joint

        self.AI_wall_init_pose = [
            [233.034, -137.454, 54.061, 148.394, 179.928, -121.78],
            [232.998, -77.359, 54.039, 148.814, 179.924, -121.369],
            [232.963, -17.258, 54.027, 148.992, 179.923, -121.209],
            [231.999, 42.815, 54.024, 150.607, 179.921, -119.603],
            [231.01, 102.933, 54.014, 152.954, 179.919, -117.273],
            [230.446, 162.316, 54.292, 123.353, 179.93, -146.878]
        ]
        self.player_wall_init_pose = [
            [563.29, 257.90, 58.06, 99.04, -179.72, 98.55],
            [533.29, 257.90, 58.06, 99.04, -179.72, 98.55],
            [503.29, 257.90, 58.06, 99.04, -179.72, 98.55],
            [473.29, 257.90, 58.06, 99.04, -179.72, 98.55],
            [443.29, 257.90, 58.06, 99.04, -179.72, 98.55],
            [413.29, 257.90, 58.06, 99.04, -179.72, 98.55]
        ]

        self.AI_pawn_init_pose = [292.34, 9.96, 69.72, 41.50, -179.47, 41.56]
        self.player_pawn_init_pose = [654.72, 16.22, 67.54, 50.38, -179.66, 50.56]

        # ------------Futures--------------
        self._clean_up_initialized = False
        self._clean_up_started = False
        self.motion_goal_future = None
        self.motion_result_future = None
        self.vision_future = None

        self.main_timer = None
        self.robot_timer = None

        self.get_logger().info("CleanUpNode ready")

    def log(self, text):
        msg = String()
        msg.data = text
        self.get_logger().info(text)
    # ==================================================
    # Service Entry
    # ==================================================
    def on_cleanup_request(self, request, response):
        self.get_logger().info("Cleanup requested")

        # 하고 있는데 또 받는 거 방지용
        if self._active:
            response.success = False
            response.message = "Cleanup already running"
            return response
        
        self.wall_used = request.wall_used

        self.log(f"request received. wall_used = {self.wall_used}")
        self._active = True
        self._response = response
        self._clean_up_initialized = False
        self.motion_goal_future = None
        self.motion_result_future = None
        self.vision_future = None
        self.main_timer = self.create_timer(0.1, self.plan_clean_up_motion)
        response.success = True
        return response

    # ==================================================
    # Step 1: Move to detection pose
    # ==================================================
    def plan_clean_up_motion(self):
        if not self._clean_up_initialized:
            self._clean_up_initialized = True
            self._clean_up_started = False 
            self.log("Clean up → moving to camera pose first.")
            # 로봇을 camera_pose로 이동시키기 위한 motion 생성
            motion = {
                'sequence': [
                    {'primitive': 'movej_pose', 'pose': self.camera_pose}
                ]
            }
            goal = self.build_motion_goal(motion)
            self.motion_goal_future = self.motion_client.send_goal_async(goal)
            return  # motion이 완료될 때까지 대기

        # --- Step 2: Goal 완료 확인 및 Result Future 생성 ---
        if self.motion_goal_future and self.motion_goal_future.done():
            goal_handle = self.motion_goal_future.result()  # ClientGoalHandle
            self.motion_goal_future = None

            # 예외처리
            if not goal_handle.accepted:
                self.log("Motion goal rejected → ERROR")
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
                    req = GetBoardState.Request()
                    req.now_state = "CLEAN_UP"
                    self.vision_future = self.vision_client.call_async(req)
            else:
                # 예외처리
                self.log("Motion failed → ERROR")
            return

        # --- Step 4: Vision 결과 처리 ---
        if self.vision_future and self.vision_future.done() and not self._clean_up_started:
            res = self.vision_future.result() #clean_board_state가 올 것
            self.vision_future = None

            self.board_state = [row.data for row in res.board_state]
            self.seq_list = []
            self.sorted_walls = 0

            for cmd in self.board_state:
                motion_seq = self.create_motion_sequence(cmd)
                self.seq_list.append(motion_seq)


        if self.seq_list and not self._clean_up_started:
            self._clean_up_started = True
            self.now_obj = 0
            self.obj_cnt = len(self.seq_list)
            if self.obj_cnt == 0:
                self.log("Nothing to clean → finish")
                self._clean_up_started = False
                return
            
            # 🔴 main 타이머 끄기
            if self.main_timer:
                self.main_timer.cancel()
                self.main_timer = None

            # 🟢 robot 타이머 켜기
            self.robot_timer = self.create_timer(
                0.1, self.robot_motion_execute
            )


    def robot_motion_execute(self):
        if not self._clean_up_started:
            return
        if not self.seq_list:
            self.log("No motion sequence → abort cleanup")
            self._clean_up_started = False
            self.robot_timer.cancel()
            return
        
        if self.now_obj >= len(self.seq_list):
            self.log("Index overflow → cleanup finished safely")
            self._clean_up_started = False
            self.robot_timer.cancel()
            return
        
        robot_motion = self.seq_list[self.now_obj]
        if self.motion_goal_future is None:
            goal = self.build_motion_goal(robot_motion)
            self.motion_goal_future = self.motion_client.send_goal_async(goal)
            return

        if self.motion_goal_future.done() and self.motion_result_future is None:
            goal_handle = self.motion_goal_future.result()
            if not goal_handle.accepted: # 에러 예외처리
                self.motion_goal_future = None
                return
            self.motion_result_future = goal_handle.get_result_async()
            return

        if self.motion_result_future and self.motion_result_future.done():
            result = self.motion_result_future.result().result
            self.motion_goal_future = None
            self.motion_result_future = None

            if result.success:
                self.log("하나 정리 끝")
                if self.now_obj == self.obj_cnt - 1:
                    self.log(f"End Cleaning")

                    msg = Bool()
                    msg.data = True
                    self.end_pub.publish(msg)

                    self.seq_list = []
                    self._clean_up_started = False
                    self._active = False
                    if self.robot_timer:
                        self.robot_timer.cancel()
                        self.robot_timer = None
                    return
                self.now_obj += 1

    def create_motion_sequence(self, cmd):            
            obj = cmd[0]
            # 출발지cell
            pos_b = cmd[1:]

            # print(cmd)

            if abs(obj)==1: # AI/Player pawn
                # 출발지cell(아래)
                pos_orig = self.set_pawn_pose(*pos_b)
                # 목적지
                if obj == -1:
                    pos_dst = self.set_pawn_pose(self.AI_pawn_init_pose[0], self.AI_pawn_init_pose[1])
                elif obj == 1: 
                    pos_dst = self.set_pawn_pose(self.player_pawn_init_pose[0], self.player_pawn_init_pose[1])
                # 출발지cell (위)
                pos_orig_up = pos_orig.copy()
                pos_orig_up[2] += 60
                pos_dst_up = pos_dst.copy()
                pos_dst_up[2] += 60
                pos_dst[2] += 10
                motion = {
                    'sequence': [
                        {'primitive': 'operate_gripper', 'width': 450},
                        {'primitive': 'movej_pose', 'pose': pick_pose},
                        {'primitive': 'movel_pose', 'pose': pos_orig_up},
                        {'primitive': 'movel_pose', 'pose': pos_orig},
                        {'primitive': 'operate_gripper', 'width':350},
                        {'primitive': 'movel_pose', 'pose': pos_orig_up},
                        {'primitive': 'movel_pose', 'pose': pos_dst_up},
                        {'primitive': 'movel_pose', 'pose': pos_dst},
                        {'primitive': 'force_control'},
                        {'primitive': 'operate_gripper', 'width': 450},
                        {'primitive': 'movel_pose', 'pose': pos_dst_up},
                        {'primitive': 'movej_pose', 'pose': pick_pose}
                    ]
                }
            # elif abs(obj)==2: # vertical, horizontal walls
            else: # Walls 3types
                # 출발지 아래
                if obj == -2:
                    pos_orig = self.set_wall_pose(*pos_b, "horizontal")
                elif obj == 2:
                    pos_orig = self.set_wall_pose(*pos_b, "vertical")
                elif obj == 3:
                    if len(pos_b) < 3:
                        print("error")
                        return
                    angle = pos_b[2]
                    pos_b_xy = pos_b[:2]
                    pos_orig = self.set_wall_pose(*pos_b_xy, "misaligned", angle)
                
                self.get_logger().info(f"sorted={self.sorted_walls}, used={self.wall_used}")
                if self.sorted_walls < self.wall_used:
                    self.get_logger().info(f"ai wall = ({self.AI_wall_init_pose[self.sorted_walls][0]},{self.AI_wall_init_pose[self.sorted_walls][1]}")
                    pos_dst = self.set_wall_pose(self.AI_wall_init_pose[self.sorted_walls][0], self.AI_wall_init_pose[self.sorted_walls][1], "vertical")
                elif self.sorted_walls >= self.wall_used:
                    self.get_logger().info(f"player wall = ({self.player_wall_init_pose[self.sorted_walls - self.wall_used][0]},{self.player_wall_init_pose[self.sorted_walls - self.wall_used][1]}")
                    pos_dst = self.set_wall_pose(self.player_wall_init_pose[self.sorted_walls - self.wall_used][0], self.player_wall_init_pose[self.sorted_walls - self.wall_used][1], "horizontal")
                # 출발지cell (위)
                pos_orig_up = pos_orig.copy()
                pos_orig_up[2] += 60
                pos_dst_up = pos_dst.copy()
                pos_dst_up[2] += 60
                pos_dst[2] += 10
                motion = {
                    'sequence': [
                        {'primitive': 'operate_gripper', 'width': 350},
                        {'primitive': 'movej_pose', 'pose': pick_pose},
                        {'primitive': 'movel_pose', 'pose': pos_orig_up},
                        {'primitive': 'movel_pose', 'pose': pos_orig},
                        {'primitive': 'operate_gripper', 'width':250},
                        {'primitive': 'movel_pose', 'pose': pos_orig_up},
                        {'primitive': 'movel_pose', 'pose': pos_dst_up},
                        {'primitive': 'movel_pose', 'pose': pos_dst},
                        {'primitive': 'force_control'},
                        {'primitive': 'operate_gripper', 'width': 350},
                        {'primitive': 'movel_pose', 'pose': pos_dst_up},
                        {'primitive': 'movej_pose', 'pose': pick_pose}
                    ]
                }
                self.sorted_walls += 1
                self.get_logger().info(f"sorted walls = {self.sorted_walls}")
                
            # print(motion)

            return motion


    def set_pawn_pose(self, x, y):
        x = float(x)
        y = float(y)
        z = 70.0
        rx, ry, rz = map(float, pick_pose_l[3:])

        return [x, y, z, rx, ry, rz]



    def set_wall_pose(self, x, y, orientation, angle=None):
        x = float(x)
        y = float(y)
        z = 50.0
        rx, ry, rz = pick_pose_l[3:]

        if orientation == "horizontal":
            rz += 90
        elif orientation == "misaligned": 
            if angle is None:
                raise ValueError("misaligned wall requires angle")

            # 🔑 핵심: angle 그대로 joint6에 반영
            # YOLO angle 기준 = wall 방향
            rz += (180 - angle)

            # 안정성: [-180, 180] 정규화
            if rz > 180.0:
                rz -= 360.0
            elif rz < -180.0:
                rz += 360.0

        return list(map(float, [x, y, z, rx, ry, rz]))

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

def main(args=None):
    rclpy.init(args=args)
    node = CleanUpNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
