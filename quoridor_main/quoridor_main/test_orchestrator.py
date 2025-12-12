#!/usr/bin/env python3
# coding: utf-8

"""
테스트용 오케스트레이터
Wakeup word 신호를 받아서 동작 확인
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class TestOrchestrator(Node):
    """Wakeup word 테스트용 오케스트레이터"""

    def __init__(self):
        super().__init__("test_orchestrator")

        # Parameters
        self.declare_parameter("wakeup_topic", "/quoridor/wakeup")
        self.declare_parameter("status_topic", "/quoridor/orch_status")
        
        wakeup_topic = self.get_parameter("wakeup_topic").value
        status_topic = self.get_parameter("status_topic").value

        # Subscriber - Wakeup word 신호 수신
        self.wakeup_sub = self.create_subscription(
            Bool,
            wakeup_topic,
            self.wakeup_callback,
            10
        )

        # Publisher - 상태 출력용
        self.status_pub = self.create_publisher(String, status_topic, 10)

        # 상태
        self.wakeup_received = False
        self.wakeup_count = 0

        self.get_logger().info("="*60)
        self.get_logger().info("🧪 Quoridor 테스트 오케스트레이터 시작")
        self.get_logger().info(f"   구독 토픽: {wakeup_topic}")
        self.get_logger().info(f"   발행 토픽: {status_topic}")
        self.get_logger().info("   Wakeup word 신호 대기 중...")
        self.get_logger().info("="*60)
        
        self.log("대기 중: '헤이 쿼리'를 말하세요")

    def log(self, text):
        """상태 로그 (콘솔 + 토픽 발행)"""
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def wakeup_callback(self, msg):
        """Wakeup word 신호 수신 콜백"""
        if msg.data:
            self.wakeup_count += 1
            self.wakeup_received = True
            
            self.log(f"✅ Wakeup 신호 수신! (#{self.wakeup_count})")
            self.get_logger().info("="*60)
            self.get_logger().info(f"🔔 '헤이 쿼리' 감지됨! (총 {self.wakeup_count}회)")
            self.get_logger().info("="*60)
            
            # TODO: 여기에 실제 게임 시작 로직 추가
            # 예시:
            # - 게임 상태를 IDLE → READY로 변경
            # - 음성 인식 서비스 활성화
            # - 사용자에게 "준비되었습니다" 음성 피드백
            # - LED 색상 변경 (대기 → 준비)
            
            self.log("→ 게임 시스템 활성화 준비 (실제 로직 여기에 추가)")
            self.log("→ 다음 단계: 음성 명령 대기 또는 게임 시작")


def main(args=None):
    rclpy.init(args=args)

    try:
        node = TestOrchestrator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n👋 테스트 오케스트레이터 종료")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()