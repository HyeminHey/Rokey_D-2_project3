#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
from rclpy.action import ActionClient

from qulido_robot_msgs.srv import GetBoardState
from qulido_robot_msgs.msg import MotionPrimitive, MotionSequence
from qulido_robot_msgs.action import ExecuteMotion

from game_orchestrator_node import GameOrchestratorNode as orch


pick_pose = [0.004, -15.49, 103.192, 0.041, 92.317, 90.012] # joint
pick_pose_l = [257.256, 7.46, 194.059, 159.331, 179.928, -110.819]


class CleanUpNode(Node):

    def __init__(self):
        super().__init__('clean_up_node')

        # ---------- Service Server ----------
        self.srv = self.create_service(
            Trigger,
            '/clean_up',
            self.on_cleanup_request
        )

        # ---------- Clients ----------
        self.vision_client = self.create_client(GetBoardState, "/vision/get_board_state")
        self.motion_client = ActionClient(self, ExecuteMotion, "/execute_motion")

        # ---------- State ----------
        self._active = False
        self._response = None

        # ---------- Cleanup Data ----------
        self.detected_pawns = []
        self.detected_walls = []

        self.current_phase = None   # "PAWN" | "WALL"
        self.current_idx = 0

        # ---------- Poses (TODO: 실제 값) ----------
        self.detection_pose = None
        self.pawn_origin_pose = None
        self.wall_origin_pose = None
        self.camera_pose = [-29.062, -49.05, 96.263, 13.934, 106.247, 68.124] # joint

        self.create_timer(0.1, self.plan_clean_up_motion)
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

        self._active = True
        self._response = response

        self.plan_clean_up_motion()
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
            goal = orch.build_motion_goal(motion)
            self.motion_goal_future = self.motion_client.send_goal_async(goal)
            return  # motion이 완료될 때까지 대기

        # --- Step 2: Goal 완료 확인 및 Result Future 생성 ---
        if self.motion_goal_future and self.motion_goal_future.done():
            goal_handle = self.motion_goal_future.result()  # ClientGoalHandle
            self.motion_goal_future = None

            # # 예외처리
            # if not goal_handle.accepted:
            #     self.log("Motion goal rejected → ERROR")
            #     self.state = OrchestratorState.ERROR
            #     return

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
                # # 예외처리
                # self.log("Motion failed → ERROR")
                # self.state = OrchestratorState.ERROR
                pass
            return

        # --- Step 4: Vision 결과 처리 ---
        if self.vision_future and self.vision_future.done() and not self._clean_up_started:
            res = self.vision_future.result()
            self.vision_future = None

            ###sequence 제작


            self._clean_up_started = True


            self.log(f"Human Turn started, prev_state saved: {self.prev_state}")



    def create_motion_sequence(self, cmd):
            seq_list = []
            
            obj = cmd[0]
            pos_b = cmd[1:]

            # print(cmd)

            if obj==-1: # AI pawn
                pos = self.set_pawn_pose(*pos_b)
                pos_obj
                pos[2] += 60

                motion = {
                    'sequence': [
                        {'primitive': 'operate_gripper', 'width': 450},
                        {'primitive': 'movej_pose', 'pose': pick_pose},
                        {'primitive': 'movel_pose', 'pose': pos},
                        {'primitive': 'movel_pose', 'pose': pos_obj},
                        {'primitive': 'operate_gripper', 'width':350},
                        {'primitive': 'movel_pose', 'pose': pos_obj_pre},
                        {'primitive': 'movel_pose', 'pose': pos_pre},
                        {'primitive': 'movel_pose', 'pose': pos},
                        {'primitive': 'force_control'},
                        {'primitive': 'operate_gripper', 'width': 450},
                        {'primitive': 'movel_pose', 'pose': pos_pre},
                        {'primitive': 'movej_pose', 'pose': pick_pose}
                    ]
                }

                
                for state in self.game_state:
                    if state[0] == -1:
                        pos_obj_b = state[1:]
                        break
                pos = self.pawn_board_to_base(*pos_b)
                pos_obj = self.pawn_board_to_base(*pos_obj_b)
                pos_pre = pos.copy()
                pos_pre[2] += 60
                pos_obj_pre = pos_obj.copy()
                pos_obj_pre[2] += 60
                pos[2] += 10
                motion = {
                    'sequence': [
                        {'primitive': 'operate_gripper', 'width': 450},
                        {'primitive': 'movej_pose', 'pose': pick_pose},
                        {'primitive': 'movel_pose', 'pose': pos_obj_pre},
                        {'primitive': 'movel_pose', 'pose': pos_obj},
                        {'primitive': 'operate_gripper', 'width':350},
                        {'primitive': 'movel_pose', 'pose': pos_obj_pre},
                        {'primitive': 'movel_pose', 'pose': pos_pre},
                        {'primitive': 'movel_pose', 'pose': pos},
                        {'primitive': 'force_control'},
                        {'primitive': 'operate_gripper', 'width': 450},
                        {'primitive': 'movel_pose', 'pose': pos_pre},
                        {'primitive': 'movej_pose', 'pose': pick_pose}
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
                pos_pre[2] += 60
                pos_obj_pre = pos_obj.copy()
                pos_obj_pre[2] += 60
                pos[2] += 10
                motion = {
                    'sequence': [
                        {'primitive': 'operate_gripper', 'width': 350},
                        {'primitive': 'movej_pose', 'pose': pick_pose},
                        {'primitive': 'movel_pose', 'pose': pos_obj_pre},
                        {'primitive': 'movel_pose', 'pose': pos_obj},
                        {'primitive': 'operate_gripper', 'width': 250},
                        {'primitive': 'movel_pose', 'pose': pos_obj_pre},
                        {'primitive': 'movel_pose', 'pose': pos_pre},
                        {'primitive': 'movel_pose', 'pose': pos},
                        {'primitive': 'force_control'},
                        {'primitive': 'operate_gripper', 'width': 350},
                        {'primitive': 'movel_pose', 'pose': pos_pre},
                        {'primitive': 'movej_pose', 'pose': pick_pose}
                    ]
                }
            print(motion)

            return motion


    def set_pawn_pose(self, x, y):
        x = x
        y = y
        z = 70.0
        rx, ry, rz = map(float, pick_pose_l[3:])

        return [x, y, z, rx, ry, rz]






















    def on_detection_pose_reached(self, success):
        if not success:
            self.finish(False, "Failed to reach detection pose")
            return
        self.call_detection()

    # ==================================================
    # Step 2: Detection
    # ==================================================
    def call_detection(self):
        self.get_logger().info("Running detection")
        # TODO: detect service
        self.on_detection_done_mock()

    def on_detection_done(self, result):
        """
        result.objects → [{type: 'pawn'|'wall', pose: ...}, ...]
        """
        self.detected_pawns = []
        self.detected_walls = []

        for obj in result.objects:
            if obj.type == 'pawn':
                self.detected_pawns.append(obj)
            elif obj.type == 'wall':
                self.detected_walls.append(obj)

        self.start_pawn_cleanup()

    def on_detection_done_mock(self):
        self.detected_pawns = ['pawn1', 'pawn2']
        self.detected_walls = ['wall1', 'wall2', 'wall3']
        self.start_pawn_cleanup()

    # ==================================================
    # Step 3: Pawn Cleanup
    # ==================================================
    def start_pawn_cleanup(self):
        self.get_logger().info("Starting pawn cleanup")
        self.current_phase = "PAWN"
        self.current_idx = 0
        self.cleanup_next()

    # ==================================================
    # Step 4: Wall Cleanup
    # ==================================================
    def start_wall_cleanup(self):
        self.get_logger().info("Starting wall cleanup")
        self.current_phase = "WALL"
        self.current_idx = 0
        self.cleanup_next()

    # ==================================================
    # Common cleanup executor
    # ==================================================
    def cleanup_next(self):
        target_list = (
            self.detected_pawns if self.current_phase == "PAWN"
            else self.detected_walls
        )

        if self.current_idx >= len(target_list):
            if self.current_phase == "PAWN":
                self.start_wall_cleanup()
            else:
                self.finish(True, "Cleanup completed")
            return

        obj = target_list[self.current_idx]
        self.get_logger().info(
            f"Cleaning {self.current_phase}: {obj}"
        )

        origin_pose = (
            self.pawn_origin_pose if self.current_phase == "PAWN"
            else self.wall_origin_pose
        )

        # TODO:
        # motion = self.build_cleanup_motion(obj, origin_pose)
        # self.send_motion(motion)

        self.on_object_done(True)

    def on_object_done(self, success):
        if not success:
            self.finish(False, f"{self.current_phase} cleanup failed")
            return

        self.current_idx += 1
        self.cleanup_next()

    # ==================================================
    # Finish
    # ==================================================
    def finish(self, success, message):
        self.get_logger().info(f"Cleanup finished: {message}")

        self._response.success = success
        self._response.message = message

        self._active = False
        self._response = None


def main(args=None):
    rclpy.init(args=args)
    node = CleanUpNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
