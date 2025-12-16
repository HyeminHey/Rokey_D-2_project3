#!/usr/bin/env python3
# coding: utf-8

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
import DR_init
from quoridor_main.detect_board.onrobot import RG

from qulido_robot_msgs.action import ExecuteMotion

import sys


# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
dsr_node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import (
        movel, movej, DR_FC_MOD_REL, release_compliance_ctrl, release_force,
        check_force_condition, task_compliance_ctrl, wait, posj, posx,
        set_desired_force, set_ref_coord, DR_AXIS_Z,
    )
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

########### Gripper Setup. Do not modify this area ############

gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

class RobotCtrlNode(Node):

    def __init__(self):
        super().__init__("robot_ctrl_node")

        self.server = ActionServer(
            self,
            ExecuteMotion,
            '/execute_motion',
            execute_callback=self.execute_cb
        )

        self.get_logger().info("robot_ctrl_node ready (list-based pose).")

    # ----------------------------------------------------------------------
    def execute_cb(self, goal_handle):

        # MotionSequence.sequence -> MotionPrimitive[]
        seq = goal_handle.request.sequence.sequence

        self.get_logger().info(f"Received {len(seq)} motion primitives.")

        for i, step in enumerate(seq):

            primitive = step.primitive
            self.get_logger().info(f"[{i}] Executing primitive: {primitive}")

            ok = False

            # MOVE
            if primitive == "movej_pose":
                ok = self.do_movej(step.target_pose)

            elif primitive == "movel_pose":
                ok = self.do_movel(step.target_pose)

            # GRIPPER
            elif primitive == "operate_gripper":
                ok = self.do_gripper(step.gripper_width)

            # FORCE
            elif primitive == "force_control":
                ok = self.do_force()

            else:
                self.get_logger().error(f"Unknown primitive '{primitive}'")
                ok = False

            # feedback
            fb = ExecuteMotion.Feedback()
            fb.current_step = primitive
            goal_handle.publish_feedback(fb)

            if not ok:
                self.get_logger().error(f"Primitive failed: {primitive}")
                goal_handle.abort()
                res = ExecuteMotion.Result()
                res.success = False
                return res

        # success
        goal_handle.succeed()
        res = ExecuteMotion.Result()
        res.success = True
        return res

    # ----------------------------------------------------------------------
    # PRIMITIVES
    # ----------------------------------------------------------------------
    def do_movej(self, pose_list):
        """
        pose_list: [j1, j2, j3, j4, j5, j6]
        """
        if len(pose_list) != 6:
            self.get_logger().error(f"[MOVE] Invalid pose list: {pose_list}")
            return False

        j1, j2, j3, j4, j5, j6 = pose_list
        self.get_logger().info(
            f"[MOVE] joint=({j1:.3f},{j2:.3f},{j3:.3f},{j4:.3f},{j5:.3f},{j6:.3f}) "
        )

        # TODO: replace with IK + controller
        movej(posj([j1, j2, j3, j4, j5, j6]), vel=VELOCITY, acc=ACC)
        wait(1.0)
        return True

    def do_movel(self, pose_list):
        """
        pose_list: [x, y, z, roll, pitch, yaw]
        """
        if len(pose_list) != 6:
            self.get_logger().error(f"[MOVE] Invalid pose list: {pose_list}")
            return False

        x, y, z, r, p, yaw = pose_list
        self.get_logger().info(
            f"[MOVE] xyz=({x:.3f},{y:.3f},{z:.3f}) "
            f"rpy=({r:.3f},{p:.3f},{yaw:.3f})"
        )

        # TODO: replace with IK + controller
        movel(posx([x, y, z, r, p, yaw]), vel=VELOCITY, acc=ACC)
        wait(1.0)
        return True

    def do_gripper(self, open_width):
        self.get_logger().info(f"[GRIPPER WIDTH] {open_width}")
        gripper.move_gripper(open_width)
        wait(0.4)
        return True

    def do_force(self):
        self.get_logger().info("[FORCE] Running force control routine...")
        set_ref_coord(1)
        task_compliance_ctrl(stx=[2000,2000,200,500,500,500])
        wait(0.5)
        set_desired_force(
            fd=[0,0,18,0,0,0],
            dir=[0,0,1,0,0,0],
            mod=DR_FC_MOD_REL
        )
        while not check_force_condition(DR_AXIS_Z, max=7):
            print("Y축 힘이 7N 이상이 될 때까지 대기...")
        release_force()
        release_compliance_ctrl()
        set_ref_coord(0)
        wait(0.5)
        return True


# ----------------------------------------------------------------------
def main(args=None):

    node = RobotCtrlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
