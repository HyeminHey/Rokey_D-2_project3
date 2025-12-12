#!/usr/bin/env python3
# coding: utf-8

"""
Wakeup Word Node for Quoridor Game (Korean Support)
한국어 wakeup word 감지 → ROS2 토픽으로 신호 발행
Porcupine v4.0.0 대응 버전
"""

import os
import struct
import time
import rclpy
from rclpy.node import Node
import pyaudio
import pvporcupine
from dotenv import load_dotenv
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Bool, String

# 환경 변수 로드
load_dotenv()
picovoice_api_key = os.getenv("PICOVOICE_ACCESS_KEY")


class WakeupWordNode(Node):
    """Wakeup Word 감지 노드 (한국어 지원, Porcupine v4.0.0)"""

    def __init__(self):
        super().__init__("wakeup_word_node")

        # Parameters
        self.declare_parameter("keyword_filename", "hello-query_ko_linux_v4_0_0.ppn")  # v4.0.0 파일명
        self.declare_parameter("model_filename", "porcupine_params_ko.pv")
        self.declare_parameter("sensitivity", 0.7)
        self.declare_parameter("wakeup_topic", "/quoridor/wakeup")
        self.declare_parameter("status_topic", "/quoridor/wakeup_status")
        self.declare_parameter("audio_device_index", -1)
        self.declare_parameter("language", "ko")

        keyword_filename = self.get_parameter("keyword_filename").value
        model_filename = self.get_parameter("model_filename").value
        sensitivity = self.get_parameter("sensitivity").value
        wakeup_topic = self.get_parameter("wakeup_topic").value
        status_topic = self.get_parameter("status_topic").value
        audio_device_index = self.get_parameter("audio_device_index").value
        language = self.get_parameter("language").value

        # API Key 확인
        if not picovoice_api_key:
            self.get_logger().error("❌ PICOVOICE_ACCESS_KEY 환경변수 필요")
            self.get_logger().error("   ~/.bashrc에 추가:")
            self.get_logger().error("   export PICOVOICE_ACCESS_KEY='your-key'")
            raise ValueError("PICOVOICE_ACCESS_KEY not found")

        # 패키지 경로
        try:
            package_share = get_package_share_directory('quoridor_main')
            
            # 키워드 파일 경로
            keyword_path = os.path.join(package_share, 'keywords', keyword_filename)
            if not os.path.exists(keyword_path):
                self.get_logger().error(f"❌ 키워드 파일 없음: {keyword_path}")
                self.get_logger().error(f"   Porcupine Console에서 v4.0.0 키워드 파일 다운로드:")
                self.get_logger().error(f"   https://console.picovoice.ai/")
                raise FileNotFoundError(keyword_path)
            
            # 한국어 모델 파일 경로
            model_path = None
            if model_filename:
                model_path = os.path.join(package_share, 'models', model_filename)
                if not os.path.exists(model_path):
                    self.get_logger().error(f"❌ 모델 파일 없음: {model_path}")
                    self.get_logger().error(f"   한국어 모델 파일(.pv)을 다음 위치에 배치:")
                    self.get_logger().error(f"   {package_share}/models/")
                    raise FileNotFoundError(model_path)
                self.get_logger().info(f"✅ 모델 파일: {model_filename}")
            
            self.get_logger().info(f"✅ 키워드 파일: {keyword_filename}")
            
        except Exception as e:
            self.get_logger().error(f"❌ 패키지 경로 오류: {e}")
            raise

        # Publishers
        self.wakeup_pub = self.create_publisher(Bool, wakeup_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)

        # Porcupine v4.0.0 초기화
        try:
            porcupine_params = {
                'access_key': picovoice_api_key,
                'keyword_paths': [keyword_path],
                'sensitivities': [sensitivity]
            }
            
            # 한국어 모델 파일 추가 (중요!)
            if model_path:
                porcupine_params['model_path'] = model_path
            
            self.porcupine = pvporcupine.create(**porcupine_params)
            
            # Porcupine 버전 확인
            porcupine_version = pvporcupine.__version__ if hasattr(pvporcupine, '__version__') else "Unknown"
            
            self.get_logger().info(f"✅ Porcupine 초기화 완료 (v{porcupine_version})")
            self.get_logger().info(f"   언어: {language.upper()}")
            self.get_logger().info(f"   민감도: {sensitivity}")
            self.get_logger().info(f"   샘플레이트: {self.porcupine.sample_rate}Hz")
            self.get_logger().info(f"   프레임길이: {self.porcupine.frame_length}")
            if model_path:
                self.get_logger().info(f"   모델: {os.path.basename(model_path)}")
                
        except pvporcupine.PorcupineInvalidArgumentError as e:
            self.get_logger().error(f"❌ Porcupine 초기화 실패 (버전 불일치): {e}")
            self.get_logger().error("   해결 방법:")
            self.get_logger().error("   1. pip install --upgrade pvporcupine")
            self.get_logger().error("   2. Porcupine Console에서 v4.0.0 키워드 재다운로드")
            self.get_logger().error("   3. 키워드 파일명을 *_v4_0_0.ppn으로 변경")
            raise
        except Exception as e:
            self.get_logger().error(f"❌ Porcupine 초기화 실패: {e}")
            self.get_logger().error("   API 키와 .ppn/.pv 파일을 확인하세요")
            raise

        # 오디오 스트림 초기화
        self.audio = pyaudio.PyAudio()
        try:
            stream_params = {
                'rate': self.porcupine.sample_rate,
                'channels': 1,
                'format': pyaudio.paInt16,
                'input': True,
                'frames_per_buffer': self.porcupine.frame_length,
            }
            
            # 특정 장치 지정 (파라미터로 받은 경우)
            if audio_device_index >= 0:
                stream_params['input_device_index'] = audio_device_index
                self.get_logger().info(f"   지정된 오디오 장치: #{audio_device_index}")
            
            self.stream = self.audio.open(**stream_params)
            self.get_logger().info(f"✅ 오디오 스트림 시작")
            
        except Exception as e:
            self.get_logger().error(f"❌ 오디오 스트림 실패: {e}")
            self.get_logger().error("   다른 장치로 시도:")
            self.get_logger().error("   ros2 run quoridor_game wakeup_word_node_korean --ros-args -p audio_device_index:=0")
            raise

        # 상태 변수
        self.detection_count = 0
        self.frame_count = 0
        self.last_log_time = time.time()
        self.language = language

        # 타이머 (프레임마다 체크)
        timer_period = self.porcupine.frame_length / self.porcupine.sample_rate
        self.create_timer(timer_period, self.check_wakeup)

        # 언어별 안내 메시지
        wakeup_phrase = "헤이 쿼리" if language == "ko" else "Hello Query"
        
        self.log_status("대기 중...")
        self.get_logger().info("="*60)
        self.get_logger().info(f"🎤 Wakeup Word 노드 준비 완료 ({language.upper()})")
        self.get_logger().info(f"   '{wakeup_phrase}'를 명확하게 말하세요!")
        self.get_logger().info(f"   신호 토픽: {wakeup_topic}")
        self.get_logger().info(f"   Test Orchestrator 연동 준비 완료")
        self.get_logger().info("="*60)

    def log_status(self, text):
        """상태 메시지 발행 (Test Orchestrator용)"""
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def check_wakeup(self):
        """Wakeup word 체크 (타이머 콜백)"""
        try:
            # 프레임 카운터
            self.frame_count += 1
            
            # 5초마다 동작 확인 로그
            current_time = time.time()
            if current_time - self.last_log_time > 5.0:
                wakeup_phrase = "헤이 쿼리" if self.language == "ko" else "Hello Query"
                self.get_logger().info(
                    f"🎧 '{wakeup_phrase}' 청취 중... "
                    f"(프레임: {self.frame_count}, 감지: {self.detection_count}회)"
                )
                self.last_log_time = current_time

            # 오디오 프레임 읽기
            pcm = self.stream.read(
                self.porcupine.frame_length,
                exception_on_overflow=False
            )
            pcm = struct.unpack_from("h" * self.porcupine.frame_length, pcm)

            # Porcupine 처리
            keyword_index = self.porcupine.process(pcm)

            # Wakeup word 감지
            if keyword_index >= 0:
                self.detection_count += 1
                wakeup_phrase = "헤이 쿼리" if self.language == "ko" else "Hello Query"
                
                # 신호 발행 (Test Orchestrator가 구독)
                wakeup_msg = Bool()
                wakeup_msg.data = True
                self.wakeup_pub.publish(wakeup_msg)

                # 상태 발행
                self.log_status(f"🔔 '{wakeup_phrase}' 감지! (#{self.detection_count})")
                
                # 로그
                self.get_logger().info("="*60)
                self.get_logger().info(f"🔔 WAKEUP WORD 감지! (#{self.detection_count})")
                self.get_logger().info(f"   감지어: {wakeup_phrase}")
                self.get_logger().info(f"   토픽 발행: /quoridor/wakeup → True")
                self.get_logger().info("="*60)

        except IOError as e:
            # 오디오 버퍼 오버플로우 등 일시적 오류는 무시
            if e.errno == -9981:  # Input overflowed
                pass
            else:
                self.get_logger().warn(f"오디오 읽기 오류: {e}")
        except Exception as e:
            self.get_logger().error(f"체크 오류: {e}")

    def destroy_node(self):
        """노드 종료 시 리소스 정리"""
        self.get_logger().info("🛑 Wakeup Word 노드 종료 중...")
        
        try:
            if hasattr(self, 'stream') and self.stream:
                self.stream.stop_stream()
                self.stream.close()
            if hasattr(self, 'audio') and self.audio:
                self.audio.terminate()
            if hasattr(self, 'porcupine') and self.porcupine:
                self.porcupine.delete()
        except Exception as e:
            self.get_logger().warn(f"정리 중 오류: {e}")
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = WakeupWordNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n👋 사용자 종료")
    except Exception as e:
        print(f"\n❌ 노드 오류: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()