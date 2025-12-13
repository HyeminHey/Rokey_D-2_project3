# -*- coding: utf-8 -*-
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

from quoridor_main.ai.action import ActionMovePawn, ActionPlaceWall
from quoridor_main.entities.board import Board
from quoridor_main.entities.coord import Coord
from quoridor_main.entities.wall import Wall
from quoridor_main.helpers import log
from quoridor_main import core 
from quoridor_main import config as cfg

from qulido_robot_msgs.srv import AiCompute

import os
import pygame
from pygame import Color


class BoardRosNode(Node):
    """
    ROS2에서 외부 시스템이 보내주는 board_state([[type, row, col], ...])
    형식을 받아서 Board 내부 상태를 갱신하는 노드
    """
    def __init__(self, board):
        super().__init__('board_ros_node')
        self.board = board
        self.orig_pose = Coord(6, 3)
        self.screen = board.screen
        self.last_request = []
        try:
            # Pygame이 초기화되었으므로 Color 객체 사용 가능
            self.wall_color = cfg.WALL_COLOR # config 사용 가능
        except:
            self.wall_color = None # Pygame 초기화 실패 시 대비

        # #안돌리고 가능
        ##subprocessing
        # try:
        #     script_path = os.path.join(
        #         os.path.dirname(__file__),   # 한 단계만 올라감
        #         "quoridor.py"
        #     )

        #     log(f"[ROS] Running quoridor.py : {script_path}")

        #     subprocess.Popen(["python3", script_path])

        # except Exception as e:
        #     log(f"[ERROR] Failed to run quoridor.py : {e}")

        self.srv = self.create_service(AiCompute, '/ai_agent/get_robot_move', self.board_state_callback)
        self.get_logger().info("AI service server ready: /ai_agent/get_robot_move")

        self.sub = self.create_subscription(
                    Int32,
                    '/game_level',
                    self.listener_callback,
                    10)
        self.sub
        self.get_logger().info("Level Subscriber Node is Running. Waiting for /game_level messages...")

        self.pub = self.create_publisher(String, '/game_finished', 10)

    def listener_callback(self, msg):
            level = msg.data
            self.get_logger().info(f'--- Received Game Level: {msg.data} ---')
            cfg.set_level(level)
            log(f"level = {cfg.LEVEL}")
            self.board.reset_AI()
            
    
    
    def board_state_callback(self, request:AiCompute, response):
        # self.get_logger().info(f"type(self.board) = {type(self.board)}")

        # self.get_logger().info("Received AiCompute request")
        self.last_request = list(request.added)
        # self.get_logger().info(f"Received added: {self.last_request}")
        # action = self.board_state_to_actions(self.last_request)
        if self.last_request:
                act_suc = self.board.apply_player_action(self.last_request)


        #finished
        if type(self.board.won_player) == int:
            response.ai_cmd = [0, 0, 0]
            msg = String()
            if self.board.won_player == 0:
                msg.data = "player"
            elif self.board.won_player == 1:
                msg.data = "AI"
            self.pub.publish(msg)
            self.get_logger().info(f"Game Finished Topic published. Winner is {msg.data}")
            return response

        if act_suc:
            # log(f"ai_action = {self.board.ai_action}")

            ai_act = self.board.ai_action
            if isinstance(ai_act, ActionPlaceWall):
                ai_t = -2 if ai_act.horiz else 2
                ai_r = ai_act.coord.row
                ai_c = ai_act.coord.col
            elif isinstance(ai_act, ActionMovePawn):
                ai_t = -1
                ai_r = ai_act.dest.row
                ai_c = ai_act.dest.col
            res_list = [ai_t, ai_r, ai_c]
            # response.ai_cmd = list(res_list)
            response.ai_cmd = res_list
            # self.get_logger().info(f"Responding ai_cmd: {response.ai_cmd}")

            return response
        # error, try again
        elif not act_suc:
            response.ai_cmd = [0, 1, 1]
            return response

    # def board_state_to_actions(self, player_move_state, last_pawn_positions=None):

    #     action = None
    #     t = player_move_state[0]
    #     r = player_move_state[1]
    #     c = player_move_state[2]

    #     log(f"type = {t}, Coord = ({r}, {c})")
    #     if abs(t) == 1:
    #         if t == 1:
    #             from_ = self.orig_pose
    #             to_ = Coord(r, c)
    #             action = ActionMovePawn(from_, to_)
    #             self.orig_pose = Coord(r, c)
    #             log(f"transformed to ActionMovePawn : {action}")

    #     elif abs(t) == 2:
    #         horiz = True if t == -2 else False  
    #         wall = Wall(
    #             coord=Coord(r, c), 
    #             horiz=horiz,
    #             screen=self.screen,   # __init__에서 저장한 screen 객체
    #             board=self.board,     # 현재 노드의 board 객체
    #             color=self.wall_color # 정의한 색상
    #         )            
    #         action = ActionPlaceWall(wall)
    #         log(f"transformed to ActionPlaceWall : {action}")


    #     return action



def main(args=None):
    rclpy.init(args=args)
    core.init()
    # 1. Pygame 초기화
    # GUI를 직접 표시하지 않더라도 Board 객체 생성을 위해 Pygame을 초기화해야 합니다.
    try:
        pygame.init()
        # 2. Board 생성자에 전달할 더미 'screen' 객체 생성 (ROS 노드는 화면을 그리지 않으므로 최소 크기로 설정)
        # 이 Surface는 Board 객체가 초기화될 때 Drawable 객체들에 전달됩니다.
        dummy_screen = pygame.Surface((1, 1)) 
        #################### action객체를 만들어서 그걸로 do action하는데 그 인자인 board가 dummy라면 제대로 화면이 안움직이나? 그럼 이 코드가 필요할 것 같음 (돌아가긴함)
        # pygame.display.set_mode((800, 600))
        # pygame.display.set_caption(cfg.GAME_TITLE)
        # screen = pygame.display.get_surface()

    except Exception as e:
        log(f"[ERROR] Pygame initialization failed: {e}")
        rclpy.shutdown()
        return

    # 3. Board 객체 생성 시 dummy_screen 전달
    initial_board = Board(screen=dummy_screen)
    core.BOARD = initial_board
    node = BoardRosNode(initial_board)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        node.get_logger().error(f"now ai turn 300 line error : {e}")
    except Exception as e:
        node.get_logger().error(f"An error occurred: {e}")

    pygame.quit()
    node.destroy_node()
    rclpy.shutdown()
