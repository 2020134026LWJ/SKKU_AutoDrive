#!/usr/bin/env python3
"""
신호등 검출기 오프라인 평가 — 하드웨어 0, ROS 0.

두 가지를 잰다. 둘 다 실물 신호등 없이 지금 측정 가능하다.

1) 오검출 (false positive) — **신호등이 하나도 없는** 녹화 영상 98초에 돌린다.
   정답은 전 프레임 'None'. 하나라도 색을 뱉으면 그게 곧 오검출이다.
   (원래 로직은 여기서 71%를 뱉었다 — 주황 기둥=빨강, 잔디천=초록)

2) 미검출 (false negative) — 그 영상 위에 **가짜 신호등을 합성**해서 얹고 돌린다.
   실물 사진이 없으니 완벽한 대역은 아니지만, "밝고 둥근 점등부"라는 전제가
   코드에 제대로 구현됐는지는 확인된다. 실물 사진이 나오면 min_area/max_area만
   다시 재면 된다 (docs/CALIBRATION.md).

실행:  python3 scripts/traffic_light_eval.py
"""

import os
import sys

import cv2
import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_LIB = os.path.join(
    _ROOT, 'ros2_ws/src/AutoSDV-HPC/src/camera_perception_pkg/camera_perception_pkg/lib')
sys.path.insert(0, _LIB)

from traffic_light_lib import TrafficLightParams, TrafficLightVoter, find_lamp  # noqa: E402

VIDEO = os.path.join(_LIB, 'Collected_Datasets/driving_simulation.mp4')

# 합성 신호등 색 (BGR) — 실제 LED 점등부처럼 '밝고 진한' 색
LAMP_BGR = {'Red': (40, 40, 235), 'Yellow': (40, 220, 235), 'Green': (60, 220, 60)}


def draw_fake_traffic_light(frame, color: str, cx: int, cy: int, lamp_r: int):
    """가짜 신호등 합성 — 어두운 등기구 몸체 + 켜진 램프 1개 (세로 3구).

    실물 신호등의 특징을 그대로 흉내낸다: 검은 몸체에 박힌 밝고 둥근 점등부.
    """
    body_w, body_h = int(lamp_r * 2.8), int(lamp_r * 8.4)
    x0, y0 = cx - body_w // 2, cy - body_h // 2
    cv2.rectangle(frame, (x0, y0), (x0 + body_w, y0 + body_h), (25, 25, 25), -1)

    slots = ['Red', 'Yellow', 'Green']
    for i, slot in enumerate(slots):
        sy = y0 + int(body_h * (i + 0.5) / 3)
        if slot == color:
            cv2.circle(frame, (cx, sy), lamp_r, LAMP_BGR[slot], -1)
            # 점등부는 중심이 더 밝다 (LED 확산) — 살짝 블러로 흉내
            cv2.circle(frame, (cx, sy), max(1, lamp_r // 2), (255, 255, 255), -1)
            cv2.GaussianBlur(frame, (5, 5), 0, dst=frame)
        else:
            cv2.circle(frame, (cx, sy), lamp_r, (18, 18, 18), -1)   # 꺼진 램프
    return frame


def run_false_positive(p: TrafficLightParams):
    """신호등 없는 원본 영상 → 전 프레임 'None'이어야 한다."""
    cap = cv2.VideoCapture(VIDEO)
    voter = TrafficLightVoter(p.consec_frames)
    total = 0
    raw_hits = 0            # 프레임 단위 오검출 (투표 전)
    confirmed = {}          # 투표까지 통과한 오검출 (진짜 위험한 것)
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        total += 1
        color, _ = find_lamp(frame, p)
        if color is not None:
            raw_hits += 1
        state = voter.update(color)
        if state != 'None':
            confirmed[state] = confirmed.get(state, 0) + 1
    cap.release()
    return total, raw_hits, confirmed


def run_true_positive(p: TrafficLightParams, lamp_r: int):
    """같은 영상 + 합성 신호등 → 그 색을 찾아내야 한다."""
    results = {}
    for color in ('Red', 'Yellow', 'Green'):
        cap = cv2.VideoCapture(VIDEO)
        voter = TrafficLightVoter(p.consec_frames)
        total = hit = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            total += 1
            # 화면 위쪽에 신호등을 얹는다 (실제로 신호등이 보일 만한 위치)
            frame = draw_fake_traffic_light(frame, color, cx=470, cy=110, lamp_r=lamp_r)
            detected, _ = find_lamp(frame, p)
            if voter.update(detected) == color:
                hit += 1
        cap.release()
        results[color] = (hit, total)
    return results


def main():
    p = TrafficLightParams()
    print("=" * 68)
    print("신호등 검출기 평가 (하드웨어 없이)")
    print("=" * 68)

    print("\n[1] 오검출 — 신호등이 '없는' 녹화 영상 98초")
    total, raw, confirmed = run_false_positive(p)
    print(f"    프레임 {total}개 중")
    print(f"    프레임 단위 오검출 : {raw:5d}  ({raw / total * 100:5.2f}%)")
    if confirmed:
        for c, n in confirmed.items():
            print(f"    ★ 확정 오검출     : {c} {n}프레임 ({n / total * 100:.2f}%)  ← 차가 실제로 오동작한다")
    else:
        print(f"    ★ 확정 오검출     : 0  ({p.consec_frames}프레임 연속 투표 통과 없음)")
    print(f"    (참고: 원래 '색 비율' 로직은 여기서 71% 오검출 — 빨강 55% / 초록 16%)")

    print("\n[2] 미검출 — 같은 영상에 가짜 신호등을 합성해서 얹음")
    for lamp_r in (14, 9, 6):
        res = run_true_positive(p, lamp_r)
        line = "  ".join(f"{c} {h / t * 100:5.1f}%" for c, (h, t) in res.items())
        print(f"    램프 반지름 {lamp_r:2d}px (면적 ~{int(3.14 * lamp_r ** 2):4d}px) : {line}")

    ok = not confirmed
    print("\n" + "=" * 68)
    print("결과: " + ("통과 — 신호등 없는 영상에서 오검출 0" if ok
                     else "실패 — 오검출이 남아 있다 (임계값 조정 필요)"))
    print("=" * 68)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
