# -*- coding: utf-8 -*-
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
import threading

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
        self.screen = board.screen
        self.last_request = []

        # #subprocessing
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

        self.level_sub = self.create_subscription(
                    Int32,
                    '/game_level',
                    self.level_listener_callback,
                    10)
        self.level_sub
        self.get_logger().info("Level Subscriber Node is Running. Waiting for /game_level messages...")

        self.setting_sub = self.create_subscription(
                    String,
                    '/now_game_state',
                    self.state_listener_callback,
                    10)
        self.setting_sub
        self.get_logger().info("Setting Subscriber Node is Running. Waiting for /clean_up messages...")


        self.pub = self.create_publisher(String, '/game_finished', 10)

    def reset_game(self):
        # initializing
        core.init()
        pygame.init()

        pygame.display.set_mode((1200, 800))
        pygame.display.set_caption(cfg.GAME_TITLE)
        screen = pygame.display.get_surface()

        screen.fill(Color(75, 75, 75))
        board = core.BOARD = Board(screen)
        board.draw()
        log('System initialized OK')

        self.board = board
        self.screen = self.board.screen
        self.last_request = []


    def level_listener_callback(self, msg):
        if type(self.board.won_player) == int:
            self.reset_game()

        level = msg.data
        self.get_logger().info(f'--- Received Game Level: {msg.data} ---')
        cfg.set_level(level)
        log(f"level = {cfg.LEVEL}")
        self.board.reset_AI()

    def state_listener_callback(self, msg):
        self.now_state = str(msg.data)

        if self.now_state == "WAIT_WAKE":
            # State Box
            state_rect = pygame.Rect(600, 600, 560, 130)
            pygame.draw.rect(self.screen, cfg.STATE_BOX_COLOR, state_rect, 0)
            self.board.msg(650, 635, "Say \"Hey, Quori\"", fsize=cfg.STATE_BOX_FONT_SIZE - 20)
        
        elif self.now_state == "WAIT_START":
            # State Box
            state_rect = pygame.Rect(600, 600, 560, 130)
            pygame.draw.rect(self.screen, cfg.STATE_BOX_COLOR, state_rect, 0)
            self.board.msg(650, 635, "Say \"Start Game\"", fsize=cfg.STATE_BOX_FONT_SIZE - 20)        
        
        elif self.now_state == "HUMAN_TURN":
            # State Box
            state_rect = pygame.Rect(600, 600, 560, 130)
            pygame.draw.rect(self.screen, cfg.STATE_BOX_COLOR, state_rect, 0)
            self.board.msg(650, 620, "Now Detecting...", fsize=cfg.STATE_BOX_FONT_SIZE - 30)
            self.board.msg(850, 675, "Don't Touch!", fsize=cfg.STATE_BOX_FONT_SIZE - 40)
            pygame.display.flip()
        
        elif self.now_state == "RULE_BREAK":
           # State Box
            state_rect = pygame.Rect(600, 600, 560, 130)
            pygame.draw.rect(self.screen, cfg.STATE_BOX_COLOR, state_rect, 0)
            self.board.msg(655, 620, "Rule Violation", fsize=cfg.STATE_BOX_FONT_SIZE - 30)
            self.board.msg(850, 675, "Resetting...", fsize=cfg.STATE_BOX_FONT_SIZE - 40)
            pygame.display.flip()

        elif self.now_state == "PRE_DET":
            # State Box
            state_rect = pygame.Rect(600, 600, 560, 130)
            pygame.draw.rect(self.screen, cfg.STATE_BOX_COLOR, state_rect, 0)
            self.board.msg(650, 635, "-- Your Turn --", fsize=cfg.STATE_BOX_FONT_SIZE)

        elif self.now_state == "ROBOT_THINK":
            pass
        
        elif self.now_state == "ROBOT_PLAN":
            pass
        
        elif self.now_state == "ROBOT_EXECUTE":
            # State Box
            state_rect = pygame.Rect(600, 600, 560, 130)
            pygame.draw.rect(self.screen, cfg.STATE_BOX_COLOR, state_rect, 0)
            self.board.msg(680, 640, "Now Moving...", fsize=cfg.STATE_BOX_FONT_SIZE - 15)
        
        elif self.now_state == "CLEAN_UP":
            # State Box
            state_rect = pygame.Rect(600, 600, 560, 130)
            pygame.draw.rect(self.screen, cfg.STATE_BOX_COLOR, state_rect, 0)
            self.board.msg(665, 640, "Now Cleaning...", fsize=cfg.STATE_BOX_FONT_SIZE - 15)
        


    def board_state_callback(self, request:AiCompute, response):
        # self.get_logger().info(f"type(self.board) = {type(self.board)}")

        # self.get_logger().info("Received AiCompute request")
        self.last_request = list(request.added)
        # self.get_logger().info(f"Received added: {self.last_request}")
        if self.last_request:
                act_suc = self.board.apply_player_action(self.last_request)

        # finished
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


        if act_suc: # AI movement True
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
            response.ai_cmd = res_list
            # self.get_logger().info(f"Responding ai_cmd: {response.ai_cmd}")

            return response
        
        # error, try again
        elif not act_suc:
            response.ai_cmd = [0, 0, -1]
            return response


def main(args=None):
    rclpy.init(args=args)
    
    core.init()
    pygame.init()

    clock = pygame.time.Clock()
    # pygame.display.set_mode((800, 600))
    pygame.display.set_mode((1200, 800))
    pygame.display.set_caption(cfg.GAME_TITLE)
    screen = pygame.display.get_surface()

    screen.fill(Color(75, 75, 75))
    board = core.BOARD = Board(screen)
    board.draw()
    log('System initialized OK')

    node = BoardRosNode(board)
    
    try:
        while rclpy.ok():
            # 1. ROS2 메시지 처리 (매우 짧은 시간만 허용)
            rclpy.spin_once(node, timeout_sec=0.01) # 타임아웃을 더 짧게 설정하여 GUI 응답성 개선   

            # 2. AI 턴 처리 (백그라운드 스레드)
            if not board.computing and not board.finished:
                if board.current_player.AI:
                    board.computing = True
                    # AI 계산은 반드시 별도 스레드로 처리하여 GUI를 Block하지 않도록 합니다.
                    thread = threading.Thread(target=board.computer_move)
                    thread.start()
            
            # 3. Pygame 이벤트 처리 (가장 중요)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt # 파이게임 창 닫기 버튼 처리

            # 4. 화면 갱신
            clock.tick(cfg.FRAMERATE)
            pygame.display.flip()     

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Exception: {e}")

    finally:
        pygame.quit()
        print("Shutting down...")
        node.destroy_node()
        rclpy.shutdown()