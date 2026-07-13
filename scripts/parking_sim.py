#!/usr/bin/env python3
"""주차 FSM 검증용 가짜 환경 — 하드웨어 없이 돈다.

카메라도 아두이노도 없이 주차 시퀀스를 끝까지 돌려본다. 주차칸 뒤 경계선을 하나
가정하고, 차가 실제로 발행한 제어 명령(topic_control_signal)을 적분해서 선까지의
거리를 계산한 뒤 parking_rear_distance로 되먹인다. 즉 폐루프가 진짜로 닫힌다.

topic_control_signal을 구독하는 이유: parking_controller의 parking_command를 바로
보면 motion_planner의 양보(relay)가 동작하는지 검증이 안 된다. 최종 출력을 봐야
"주차 분기가 라이다 정지 분기를 제대로 이겼는지"까지 확인된다.

[핵심] 카메라는 뒷선을 **항상 볼 수 있는 게 아니다.** 이 하네스의 존재 이유가 그것:

    거리 |  1.2m ~~~~~~~~ 0.25m ~~~~~ 0m
         |  [안 보임]     [보임]      [안 보임]
         |  옆차가 가림    폐루프 구간   범퍼 밑 사각지대

    "안 보임"을 뭉뚱그려 ABORT하면 **정상 주차가 끝날 때마다 중단된다.**
    FSM은 세 가지를 구분해야 한다: 아직 못 봄 / 가까이서 잃음 / 멀리서 잃음.

실행:
    source scripts/setup_env.sh
    ros2 run decision_making_pkg parking_controller_node    # 터미널 1
    ros2 run decision_making_pkg motion_planner_node        # 터미널 2
    python3 scripts/parking_sim.py                          # 터미널 3

시나리오:
    --scenario normal      정상 주차 — 가려짐→보임→사각지대를 전부 거친다 → DONE
    --scenario occluded    옆차에 오래 가려 뒷선이 늦게 보인다 → 찾다가 결국 DONE
    --scenario never_seen  뒷선이 끝내 안 보인다 → ABORT (칸을 잘못 짚은 것)
    --scenario camera_lost 멀리서 카메라가 죽는다 → ABORT (계속 후진하면 안 된다)
    --scenario timeout     선이 아주 멀다 → 제한시간 초과 ABORT

[주의] 여기 속도-거리 변환(SPEED_TO_MPS)은 임의값이다. 이 하네스는 "FSM이 설계대로
흐르는가"를 보는 것이지 실제 주차 정확도를 예측하지 않는다. 실측은 하드웨어에서.
"""

import argparse
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String
from interfaces_pkg.msg import MotionCommand

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rear_park_eval as RPE   # noqa: E402  (합성 후방 장면 렌더러를 공유)

TICK = 0.05  # [s] 시뮬레이션 스텝 (20Hz)
SPEED_TO_MPS = 0.003  # PWM 1당 m/s (임의값. PWM 70 → 0.21 m/s)
INITIAL_LINE_DIST = 1.20  # [m] 시작 시 뒤 경계선까지 거리
TRIGGER_AFTER = 1.0  # [s] 이만큼 주행하다가 주차 트리거를 쏜다

# --- 카메라 가시 구간 모델 (이 하네스의 핵심) ---
VISIBLE_FROM = 0.90   # [m] 이보다 멀면 아직 옆차에 가려 안 보인다
BLIND_DIST = 0.25     # [m] 이보다 가까우면 범퍼 밑 사각지대 → 안 보인다
                      # (FSM의 blind_enter_dist와 같은 값이어야 의미가 맞는다)
OCCLUDED_FROM = 0.70  # [m] occluded 시나리오: 이만큼 들어와야 비로소 보인다.
                      # 옆차 가림은 주로 '비스듬히 접근할 때' 생긴다. 칸 축과 나란해지면
                      # (REVERSE_STRAIGHT) 카메라가 칸 안을 곧장 보므로 곧 보이기 시작한다.
                      # 이 값이 FSM의 blind_search_max_dist 예산 안에 들어와야 주차가 된다.
CAMERA_LOST_AT = 0.60  # [m] camera_lost 시나리오: 이 거리에서 카메라가 죽는다


class ParkingSim(Node):
    def __init__(self, scenario, camera=False):
        super().__init__('parking_sim')
        self.scenario = scenario
        # camera=True → 거리를 직접 쏘지 않고 **후방 영상을 그려서 발행**한다.
        # 그러면 실제 rear_park_detector_node가 그 영상을 보고 거리를 계산해 FSM에 넘긴다.
        # = 카메라 → 인식 → 판단 → 제어 전체 체인이 진짜로 돈다 (통합 테스트).
        self.camera = camera

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        self.line_dist = 20.0 if scenario == 'timeout' else INITIAL_LINE_DIST
        self.camera_alive = True
        self.elapsed = 0.0
        self.triggered = False
        self.state = 'IDLE'
        self.last_state = None
        self.speed = 0
        self.was_visible = False

        self.create_subscription(MotionCommand, 'topic_control_signal', self.command_callback, qos)
        self.create_subscription(String, 'parking_state', self.state_callback, qos)

        self.distance_pub = self.create_publisher(Float32, 'parking_rear_distance', qos)
        self.image_pub = self.create_publisher(Image, 'image_02', qos)
        self.trigger_pub = self.create_publisher(Bool, 'parking_trigger', qos)

        self.create_timer(TICK, self.tick)
        mode = '카메라 인 더 루프 (영상 발행 → 실제 인식 노드가 거리 계산)' if camera \
            else '거리 직접 주입 (FSM만 검증)'
        print(f'[sim] 시나리오={scenario}, 뒤 경계선까지 {self.line_dist:.2f} m에서 시작')
        print(f'[sim] 모드: {mode}')

    def publish_rear_image(self):
        """지금 상황을 후방 영상으로 그려서 image_02로 발행.

        '뒷선이 보이나 안 보이나'를 여기서 판정하지 않는다 — 장면만 그리고, **보이는지 아닌지는
        인식 노드가 스스로 판단한다.** 사각지대(선이 화면 밖)도 렌더링 결과로 자연히 생긴다.
        그게 이 통합 테스트의 요점이다.
        """
        show_line = True
        if self.scenario == 'never_seen':
            show_line = False                       # 칸을 잘못 짚어 뒷선이 아예 없다
        elif self.scenario == 'occluded' and self.line_dist > OCCLUDED_FROM:
            show_line = False                       # 아직 옆차에 완전히 가려 있다

        img = RPE.render(self.line_dist, RPE.IDENT, show_line=show_line)
        msg = Image()
        msg.height, msg.width = img.shape[0], img.shape[1]
        msg.encoding = 'bgr8'
        msg.step = img.shape[1] * 3
        msg.data = img.tobytes()
        self.image_pub.publish(msg)

    def line_visible(self):
        """지금 후방 카메라가 뒤 경계선을 볼 수 있나."""
        if not self.camera_alive:
            return False
        if self.line_dist <= BLIND_DIST:
            return False                      # 범퍼 밑 사각지대
        if self.scenario == 'never_seen':
            return False                      # 끝내 못 봄 (칸을 잘못 짚음)
        near = OCCLUDED_FROM if self.scenario == 'occluded' else VISIBLE_FROM
        return self.line_dist <= near         # 그보다 멀면 옆차가 가림

    def command_callback(self, msg: MotionCommand):
        self.speed = msg.left_speed

    def state_callback(self, msg: String):
        self.state = msg.data

    def tick(self):
        self.elapsed += TICK

        # 트리거는 **매 틱 계속** 발행한다. 한 번만 쏘면 유실된다 —
        # 노드가 아직 서로를 발견하기 전(DDS discovery)에 쏜 메시지는 그냥 사라지고,
        # FSM은 영원히 IDLE에 머문다. 인식 노드를 하나 더 띄웠더니 discovery가 느려져
        # 실제로 이 일이 터졌다(하네스가 무한정 멈춤). 실차에서도 언제든 재현될 수 있다.
        if self.elapsed >= TRIGGER_AFTER:
            self.trigger_pub.publish(Bool(data=True))
            if not self.triggered:
                self.triggered = True
                print(f'[sim] t={self.elapsed:4.1f}s  주차 트리거 발행 (이후 계속 유지)')

        # 차를 움직인다. 후진(speed<0)하면 뒤 경계선에 가까워진다.
        self.line_dist -= (-self.speed) * SPEED_TO_MPS * TICK
        self.line_dist = max(self.line_dist, 0.0)

        if self.scenario == 'camera_lost' and self.line_dist <= CAMERA_LOST_AT and self.camera_alive:
            self.camera_alive = False
            print(f'[sim] t={self.elapsed:4.1f}s  ** 카메라 강제 종료 (d={self.line_dist:.2f}m)')

        if self.camera:
            # 영상만 내보낸다. 거리는 rear_park_detector_node가 스스로 계산한다.
            if self.camera_alive:
                self.publish_rear_image()
        else:
            visible = self.line_visible()
            if visible:
                self.distance_pub.publish(Float32(data=float(self.line_dist)))
                if not self.was_visible:
                    print(f'[sim] t={self.elapsed:4.1f}s  뒷선 보이기 시작 (d={self.line_dist:.2f}m)')
                    self.was_visible = True
            elif self.was_visible:
                print(f'[sim] t={self.elapsed:4.1f}s  뒷선 사라짐 (d={self.line_dist:.2f}m) '
                      f'{"— 사각지대" if self.line_dist <= BLIND_DIST else "— 비정상"}')
                self.was_visible = False

        if self.state != self.last_state:
            print(f'[sim] t={self.elapsed:4.1f}s  상태={self.state:<16} d={self.line_dist:.3f}m  speed={self.speed}')
            self.last_state = self.state

        if self.line_dist <= 0.0 and self.state not in ('DONE', 'ABORT'):
            print(f'\n[sim] 실패: 뒤 경계선을 넘어감 (상태={self.state})')
            raise SystemExit(1)

        if self.state in ('DONE', 'ABORT'):
            self.report()
            raise SystemExit(0)

    def report(self):
        print()
        d = self.line_dist
        if self.scenario in ('normal', 'occluded'):
            # 사각지대를 눈감고 지난 뒤 멈춰야 한다. 선을 넘지 않고(>0), 칸 안에 들어와야(<=0.25).
            ok = self.state == 'DONE' and 0.0 < d <= BLIND_DIST
            print(f'[sim] {"통과" if ok else "실패"}: 상태={self.state}, 최종거리={d:.3f}m '
                  f'(기대: DONE — 사각지대에서 눈감고 마무리, 선은 안 넘음)')
        elif self.scenario == 'never_seen':
            ok = self.state == 'ABORT'
            print(f'[sim] {"통과" if ok else "실패"}: 상태={self.state} '
                  f'(기대: ABORT — 뒷선을 끝내 못 봤으면 밀어붙이면 안 된다)')
        elif self.scenario == 'camera_lost':
            ok = self.state == 'ABORT'
            print(f'[sim] {"통과" if ok else "실패"}: 상태={self.state}, 잃은 거리={CAMERA_LOST_AT}m '
                  f'(기대: ABORT — 사각지대보다 먼 곳에서 잃었으면 고장이다)')
        elif self.scenario == 'timeout':
            ok = self.state == 'ABORT'
            print(f'[sim] {"통과" if ok else "실패"}: 상태={self.state} (기대: ABORT — 제한시간 초과)')
        else:
            ok = False
        if not ok:
            raise SystemExit(1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', default='normal',
                        choices=['normal', 'occluded', 'never_seen', 'camera_lost', 'timeout'])
    parser.add_argument('--camera', action='store_true',
                        help='후방 영상을 발행해 실제 인식 노드까지 루프에 넣는다 (통합 테스트)')
    args = parser.parse_args()

    rclpy.init()
    node = ParkingSim(args.scenario, camera=args.camera)
    code = 0
    try:
        rclpy.spin(node)
    except SystemExit as e:
        code = e.code
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
