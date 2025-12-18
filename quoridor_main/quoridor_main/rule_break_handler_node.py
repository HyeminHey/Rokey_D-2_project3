#!/usr/bin/env python3
# coding: utf-8

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from qulido_robot_msgs.srv import FixRuleBreak, GetBoardState
from qulido_robot_msgs.action import ExecuteMotion
from qulido_robot_msgs.msg import MotionPrimitive, MotionSequence

# Board constants
BOARD_X_MIN = 270
BOARD_X_MAX = 675
BOARD_Y_MIN = -191
BOARD_Y_MAX = 214
PAWN_CELL = 45.0
PAWN_GAP  = 15.0
pick_pose = [0.004, -15.49, 103.192, 0.041, 92.317, 90.012]
pick_pose_l = [257.256, 7.46, 194.059, 159.331, 179.928, -110.819]

# TODO: 실제 로봇으로 좌표 측정 필요
# DISCARD_POSITION = [24.10, 38.55, 58.78, -0.40, 82.74, 23.75] # joint
DISCARD_POSITION = [563.29, 267.90, 50.06, 99.04, -179.72, 98.55] # task


class RuleBreakHandlerNode(Node):
    
    def __init__(self):
        super().__init__('rule_break_handler')
        
        self.callback_group = ReentrantCallbackGroup()
        
        self.declare_parameter("motion_action", "/execute_motion")
        self.declare_parameter("rule_break_service", "/fix_rule_break")
        
        self.motion_action_name = self.get_parameter("motion_action").value
        rule_break_srv = self.get_parameter("rule_break_service").value
        
        self.service = self.create_service(
            FixRuleBreak,
            rule_break_srv,
            self.handle_rule_break,
            callback_group=self.callback_group
        )
        
        self.motion_client = ActionClient(
            self, 
            ExecuteMotion, 
            self.motion_action_name,
            callback_group=self.callback_group
        )
        
        self.get_logger().info("✅ RuleBreakHandler node initialized")
    
    def handle_rule_break(self, request, response):
        try:
            self.get_logger().info("🚫 Rule break request received")
            
            prev_state = self.unflatten_state(request.prev_state_flat)
            current_state = self.unflatten_state(request.current_state_flat)
            
            # 말 위반: removed (사라진 것 = 원래 위치)
            # 벽 위반: added (추가된 것 = 잘못 놓인 벽)
            removed = self.compute_removed(prev_state, current_state)
            added = self.compute_added(prev_state, current_state)
            wrong_position = added

            self.get_logger().info(f"Wrong position: {wrong_position}")
            self.get_logger().info(f"Prev state: {prev_state}")
            self.get_logger().info(f"Current state: {current_state}")
            
            if removed:
                # 말 복구: removed가 원래 위치
                self.get_logger().info(f"Removed: {removed}")
                motion = self.plan_rule_break_motion(wrong_position, removed, is_wall=False)
            else:
                # 벽 제거: removed가 비어있으면 반대로 계산 (added)
                self.get_logger().info(f"Added: {added}")
                if not added:
                    raise Exception("Cannot find added or removed piece")
                motion = self.plan_rule_break_motion(wrong_position, added, is_wall=True)
            
            success = self.execute_motion_sync(motion)
            
            response.success = success
            if success:
                response.message = "Rule break fixed successfully"
                self.get_logger().info("✅ Rule break resolved")
            else:
                response.message = "Failed to fix rule break"
                self.get_logger().error("❌ Rule break resolution failed")
            
            return response
            
        except Exception as e:
            self.get_logger().error(f"Exception in handle_rule_break: {e}")
            import traceback
            traceback.print_exc()
            response.success = False
            response.message = f"Error: {str(e)}"
            return response
    
    def unflatten_state(self, flat_state):
        """[1,2,3, -1,4,5] → [[1,2,3], [-1,4,5]]"""
        if len(flat_state) % 3 != 0:
            self.get_logger().error(f"Invalid flat_state length: {len(flat_state)}")
            return []
        
        state = []
        for i in range(0, len(flat_state), 3):
            state.append(list(flat_state[i:i+3]))
        return state
    
    def compute_removed(self, prev, current):
        """prev에는 있는데 current에는 없는 것 (사라진 것)"""
        prev_set = set(tuple(x) for x in prev)
        current_set = set(tuple(x) for x in current)
        
        removed = list(prev_set - current_set)
        
        return list(removed[0]) if removed else []
    
    def compute_added(self, prev, current):
        """prev에는 없는데 current에는 있는 것 (추가된 것)"""
        prev_set = set(tuple(x) for x in prev)
        current_set = set(tuple(x) for x in current)
        
        added = list(current_set - prev_set)
        
        return list(added[0]) if added else []
    
    def plan_rule_break_motion(self, wrong_position, target_piece, is_wall):
        """
        target_piece: removed면 원래 위치, added면 잘못 놓인 벽
        is_wall: True면 벽 제거, False면 말 복구
        """
        wrong_pos_b = wrong_position[1:]
        
        if not is_wall:
            # 말 복구: wrong_position → target_piece 위치로
            original_pos_b = target_piece[1:]
            
            wrong_pos = self.pawn_board_to_base(*wrong_pos_b)
            original_pos = self.pawn_board_to_base(*original_pos_b)
            
            wrong_pos_pre = wrong_pos.copy()
            wrong_pos_pre[2] += 40
            original_pos_pre = original_pos.copy()
            original_pos_pre[2] += 40
            
            motion = {
                'sequence': [
                    {'primitive': 'operate_gripper', 'width': 500},
                    {'primitive': 'movej_pose', 'pose': pick_pose},
                    {'primitive': 'movel_pose', 'pose': wrong_pos_pre},
                    {'primitive': 'movel_pose', 'pose': wrong_pos},
                    {'primitive': 'operate_gripper', 'width': 350},
                    {'primitive': 'movel_pose', 'pose': wrong_pos_pre},
                    {'primitive': 'movel_pose', 'pose': original_pos_pre},
                    {'primitive': 'movel_pose', 'pose': original_pos},
                    {'primitive': 'operate_gripper', 'width': 500},
                    {'primitive': 'movel_pose', 'pose': original_pos_pre},
                    {'primitive': 'movej_pose', 'pose': pick_pose}
                ]
            }
            self.get_logger().info(f"📋 Planned: Pawn recovery from {wrong_pos_b} to {original_pos_b}")
            
        else:
            # 벽 제거: wrong_position(added) → 버리는 위치로
            if target_piece and target_piece[0] == -2:
                orientation = "horizontal"
            elif target_piece and target_piece[0] == 2:
                orientation = "vertical"
            else:
                orientation = "horizontal"
            
            # wrong_position이 추가된 벽의 위치
            wrong_pos = self.wall_board_to_base(*wrong_pos_b, orientation)
            discard_pos = DISCARD_POSITION.copy()
            
            wrong_pos_pre = wrong_pos.copy()
            wrong_pos_pre[2] += 40
            discard_pos_pre = discard_pos.copy()
            discard_pos_pre[2] += 40
            
            motion = {
                'sequence': [
                    {'primitive': 'operate_gripper', 'width': 350},
                    {'primitive': 'movej_pose', 'pose': pick_pose},
                    {'primitive': 'movel_pose', 'pose': wrong_pos_pre},
                    {'primitive': 'movel_pose', 'pose': wrong_pos},
                    {'primitive': 'operate_gripper', 'width': 240},
                    {'primitive': 'movel_pose', 'pose': wrong_pos_pre},
                    {'primitive': 'movel_pose', 'pose': discard_pos_pre},
                    {'primitive': 'movel_pose', 'pose': discard_pos},
                    {'primitive': 'operate_gripper', 'width': 350},
                    {'primitive': 'movel_pose', 'pose': discard_pos_pre},
                    {'primitive': 'movej_pose', 'pose': pick_pose}
                ]
            }
            self.get_logger().info(f"📋 Planned: Wall removal ({orientation}) from {wrong_pos_b}")
        
        return motion
    
    def execute_motion_sync(self, motion):
        import time
        
        goal = self.build_motion_goal(motion)
        
        self.get_logger().info("Waiting for motion action server...")
        if not self.motion_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available")
            return False
        
        self.get_logger().info("Sending motion goal...")
        goal_future = self.motion_client.send_goal_async(goal)
        
        # Busy wait로 대기 (executor를 블록하지 않음)
        timeout = 10.0
        start = time.time()
        while not goal_future.done():
            if time.time() - start > timeout:
                self.get_logger().error("Goal send timeout")
                return False
            time.sleep(0.01)
        
        goal_handle = goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("Motion goal rejected")
            return False
        
        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        
        # Busy wait로 결과 대기
        timeout = 40.0
        start = time.time()
        while not result_future.done():
            if time.time() - start > timeout:
                self.get_logger().error("Result timeout")
                return False
            time.sleep(0.01)
        
        result = result_future.result().result
        
        return result.success
    
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
    
    def pawn_board_to_base(self, r, c):
        """말의 보드 좌표 → 로봇 베이스 좌표"""
        x = (
            BOARD_X_MIN
            + r * (PAWN_CELL + PAWN_GAP)
            + PAWN_CELL / 2
        )

        y = (
            BOARD_Y_MIN
            + c * (PAWN_CELL + PAWN_GAP)
            + PAWN_CELL / 2
        )

        z = 70.0
        rx, ry, rz = map(float, pick_pose_l[3:])

        return [x, y, z, rx, ry, rz]
    
    def wall_board_to_base(self, r, c, orientation):
        """벽의 보드 좌표 → 로봇 베이스 좌표"""
        x = (
            BOARD_X_MIN + PAWN_CELL
            + r * (PAWN_CELL + PAWN_GAP)
            + PAWN_GAP / 2
        )

        y = (
            BOARD_Y_MIN + PAWN_CELL
            + c * (PAWN_CELL + PAWN_GAP)
            + PAWN_GAP / 2
        )

        z = 50.0
        rx, ry, rz = pick_pose_l[3:]

        if orientation == "horizontal":
            rz += 90

        return list(map(float, [x, y, z, rx, ry, rz]))


def main(args=None):
    rclpy.init(args=args)
    node = RuleBreakHandlerNode()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()