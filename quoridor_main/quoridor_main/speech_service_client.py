#!/usr/bin/env python3
# coding: utf-8

"""
Speech Service 실전 테스트 클라이언트
오케스트레이터의 게임 플로우를 시뮬레이션 (난이도 인식 포함)
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time


class GameFlowSimulator(Node):
    """게임 플로우를 시뮬레이션하는 클라이언트"""
    
    def __init__(self):
        super().__init__('game_flow_simulator')
        
        # Speech Service 클라이언트 생성
        self.speech_client = self.create_client(Trigger, '/speech_to_text')
        
        self.get_logger().info("⏳ Speech Service 연결 대기 중...")
        while not self.speech_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("   '/speech_to_text' 서비스 대기...")
        
        self.get_logger().info("✅ Speech Service 연결 완료!")
        
        # 게임 상태
        self.game_started = False
        self.difficulty = "normal"  # 게임 난이도 저장
        self.turn_count = 0
    
    def call_speech_service(self, timeout=30.0):
        """
        Speech Service 호출 (오케스트레이터의 call_speech_service와 동일)
        Returns: (success, message)
        """
        request = Trigger.Request()
        
        try:
            future = self.speech_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            
            if not future.done():
                self.get_logger().error("❌ 서비스 호출 타임아웃!")
                return False, ""
            
            response = future.result()
            
            if response is None:
                self.get_logger().error("❌ 응답이 None입니다!")
                return False, ""
            
            return response.success, response.message
        
        except Exception as e:
            self.get_logger().error(f"❌ 서비스 호출 중 오류: {e}")
            return False, ""
    
    def wait_for_game_start(self):
        """게임 시작 대기 (WAIT_START 상태 시뮬레이션)"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("🎮 게임 시작 대기 중...")
        self.get_logger().info("   '게임 시작', '시작해', '레츠고' 등을 말해주세요")
        self.get_logger().info("   난이도도 함께 말씀하시면 됩니다:")
        self.get_logger().info("   예) '게임 시작 난이도는 쉽게'")
        self.get_logger().info("   예) '어려운 난이도로 시작'")
        self.get_logger().info("="*60)
        
        while not self.game_started:
            success, message = self.call_speech_service()
            
            if not success:
                self.get_logger().warn("⚠️  음성 인식 실패, 다시 시도합니다...")
                time.sleep(1)
                continue
            
            # "start game, difficulty" 형식 파싱
            if message.startswith("start game"):
                parts = message.split(", ")
                
                if len(parts) == 2:
                    # 난이도 정보 있음
                    command, difficulty = parts
                    self.difficulty = difficulty
                    self.get_logger().info(f"✅ 게임 시작 명령 인식! (난이도: {difficulty})")
                else:
                    # 난이도 정보 없음 (이미 서버에서 normal로 설정됨)
                    self.difficulty = "normal"
                    self.get_logger().info(f"✅ 게임 시작 명령 인식! (난이도: {self.difficulty} - 기본값)")
                
                self.game_started = True
                return True
            elif message == "":
                self.get_logger().info("ℹ️  게임 명령 아님, 다시 대기합니다...")
            else:
                self.get_logger().info(f"ℹ️  예상치 못한 명령: '{message}'")
    
    def wait_for_turn_end(self):
        """턴 종료 대기 (USER_TURN 상태 시뮬레이션)"""
        self.turn_count += 1
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info(f"👤 사용자 턴 #{self.turn_count}")
        self.get_logger().info("   움직임을 완료한 후 '끝', '다음', '네 차례' 등을 말해주세요")
        self.get_logger().info("="*60)
        
        while True:
            success, message = self.call_speech_service()
            
            if not success:
                self.get_logger().warn("⚠️  음성 인식 실패, 다시 시도합니다...")
                time.sleep(1)
                continue
            
            if message == "end turn":
                self.get_logger().info("✅ 턴 종료 명령 인식!")
                return True
            elif message == "":
                self.get_logger().info("ℹ️  게임 명령 아님, 계속 대기합니다...")
            else:
                self.get_logger().info(f"ℹ️  예상치 못한 명령: '{message}'")
    
    def simulate_robot_turn(self):
        """로봇 턴 시뮬레이션 (ROBOT_TURN 상태)"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info(f"🤖 로봇 턴 시작... (난이도: {self.difficulty})")
        self.get_logger().info("   (실제로는 AI 계산 + 모션 실행)")
        self.get_logger().info("="*60)
        
        # 로봇 동작 시뮬레이션 (2초)
        for i in range(2):
            self.get_logger().info(f"   로봇 동작 중... {i+1}/2")
            time.sleep(1)
        
        self.get_logger().info("✅ 로봇 턴 완료!")
    
    def run_game_loop(self):
        """전체 게임 루프 실행"""
        print("\n" + "="*60)
        print("🎯 Quoridor Game Flow Simulator (With Difficulty)")
        print("="*60)
        print("\n이 시뮬레이터는 다음 플로우를 테스트합니다:")
        print("1. 게임 시작 대기 → '게임 시작' + 난이도 인식 (최초 1회)")
        print("   - 난이도: 쉬움(easy), 보통(normal), 어려움(hard)")
        print("   - 난이도 미지정 시 자동으로 '보통'으로 설정됩니다")
        print("2. 사용자 턴 → '턴 종료' 인식")
        print("3. 로봇 턴 (시뮬레이션)")
        print("4. 2-3 반복... (게임 종료까지)")
        print("\n종료하려면 Ctrl+C를 누르세요")
        print("="*60)
        
        input("\n▶ Enter를 눌러 시작... ")
        
        # 1. 게임 시작 대기 (최초 1회만)
        if not self.wait_for_game_start():
            self.get_logger().error("❌ 게임 시작 실패")
            return
        
        self.get_logger().info("\n" + "🎮"*30)
        self.get_logger().info(f"게임이 시작되었습니다! (난이도: {self.difficulty})")
        self.get_logger().info("이제부터 턴 종료 신호만 받습니다.")
        self.get_logger().info("🎮"*30)
        time.sleep(2)
        
        # 2. 게임 루프 (턴 종료 신호만 계속 받음)
        max_turns = 10  # 최대 10턴까지 테스트
        
        try:
            while self.turn_count < max_turns:
                # 사용자 턴
                if not self.wait_for_turn_end():
                    self.get_logger().error("❌ 턴 종료 실패")
                    break
                
                # 로봇 턴
                self.simulate_robot_turn()
                
                # 다음 턴 준비
                time.sleep(1)
                self.get_logger().info("\n" + "─"*60)
                self.get_logger().info("다음 턴으로 진행합니다...")
                self.get_logger().info("─"*60)
                time.sleep(0.5)
        
        except KeyboardInterrupt:
            self.get_logger().info("\n게임을 중단합니다...")
        
        # 게임 종료
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("🎊 게임 시뮬레이션 완료!")
        self.get_logger().info(f"   총 {self.turn_count}턴 진행됨")
        self.get_logger().info(f"   난이도: {self.difficulty}")
        self.get_logger().info("="*60)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        simulator = GameFlowSimulator()
        simulator.run_game_loop()
        
    except KeyboardInterrupt:
        print("\n\n👋 시뮬레이터를 종료합니다")
    except Exception as e:
        print(f"\n❌ 오류 발생: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()