#!/usr/bin/env python3
"""
후방 카메라 주차 인식 채점 — 하드웨어 0, ROS 0.

실차 후방 영상이 아직 없으므로 **가짜 주차장 장면을 합성**해서 채점한다.
합성이라 실물 대역은 아니지만, 다음 세 가지는 지금 확인할 수 있다:

  1) 거리를 맞게 재나        — 정답을 알고 그리므로 오차를 숫자로 낼 수 있다
  2) 옆차 가림을 견디나       — 옆차로 뒷선의 좌우를 가려도 중앙 띠로 찾아내야 한다
  3) 안 보일 때 침묵하나 ★   — 사각지대(선이 화면 밖)에서 **아무 값도 내면 안 된다.**
                              거짓 거리를 내보내면 FSM이 믿고 계속 후진한다.

실행:  python3 scripts/rear_park_eval.py
"""

import os
import sys

import cv2
import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_LIB = os.path.join(
    _ROOT, 'ros2_ws/src/AutoSDV-HPC/src/camera_perception_pkg/camera_perception_pkg/lib')
sys.path.insert(0, _LIB)

from rear_park_lib import (RearParkParams, find_lateral_error,  # noqa: E402
                           find_rear_line)

W, H = 640, 480

# 합성 장면을 '버드아이뷰 그대로' 그린다 → src_mat=dst_mat(항등변환)으로 채점.
# 실차에선 여기가 원근 영상이고 src_mat이 그걸 펴주지만, 그 4점은 카메라를 달아야
# 정해지는 값이다(docs/CALIBRATION.md). 이 하네스가 보는 건 **펴진 뒤의 로직**이다.
IDENT = RearParkParams(
    src_mat=[(0, 0), (W, 0), (W, H), (0, H)],
    dst_mat=[(0, 0), (W, 0), (W, H), (0, H)],
)


def render(dist_m, p: RearParkParams, occlude=True, lateral_px=0, show_line=True):
    """가짜 후방 주차 장면 (버드아이뷰). dist_m = 뒤 경계선까지 실제 거리.

    show_line=False → 뒷선을 아예 안 그린다 (옆차에 완전히 가려진 상황 / 칸을 잘못 짚은 상황).
    """
    img = np.full((H, W, 3), 90, np.uint8)          # 아스팔트
    img += np.random.randint(-8, 8, img.shape, dtype=np.int16).astype(np.uint8)

    # 뒤 경계선(가로 흰선). 거리 → BEV 행 (아래쪽이 차에 가깝다)
    px = (dist_m - p.cam_offset_m) / p.m_per_px
    row = int((H - 1) - px)
    if show_line and 0 <= row < H:
        cv2.rectangle(img, (0, row - 4), (W, row + 4), (235, 235, 235), -1)
    elif not show_line:
        row = -999                                  # 아래 가림 처리도 건너뛴다

    # 양옆 차 (어둡고 큰 덩어리). 칸 중앙이 lateral_px 만큼 밀려 있다고 가정.
    cx = W // 2 + lateral_px
    slot_half = 150
    cv2.rectangle(img, (0, 0), (cx - slot_half, H), (55, 50, 48), -1)
    cv2.rectangle(img, (cx + slot_half, 0), (W, H), (52, 55, 50), -1)

    if occlude and 0 <= row < H:
        # 옆차가 뒷선의 좌우 끝을 덮는다 (실제로 이렇게 가려진다)
        cv2.rectangle(img, (0, row - 8), (cx - slot_half + 40, row + 8), (55, 50, 48), -1)
        cv2.rectangle(img, (cx + slot_half - 40, row - 8), (W, row + 8), (52, 55, 50), -1)

    return img


def main():
    p = IDENT
    print("=" * 66)
    print("후방 카메라 주차 인식 채점 (합성 장면, 하드웨어 없이)")
    print("=" * 66)

    print("\n[1] 거리 정확도 — 옆차가 뒷선 좌우를 가린 상태")
    errs = []
    for truth in (0.90, 0.70, 0.55, 0.40, 0.30):
        got = find_rear_line(render(truth, p), p)
        if got is None:
            print(f"    실제 {truth:.2f}m → 못 찾음 (실패)")
            errs.append(9.99)
            continue
        err = abs(got - truth)
        errs.append(err)
        print(f"    실제 {truth:.2f}m → 측정 {got:.3f}m   오차 {err * 100:5.1f}cm")
    max_err = max(errs)

    print("\n[2] 사각지대 — 뒷선이 화면 밖으로 나감 (다 들어왔다는 신호)")
    # 거리가 cam_offset보다 작아지면 선이 BEV 아래로 사라진다
    silent = True
    for truth in (0.08, 0.04, 0.00):
        got = find_rear_line(render(truth, p), p)
        state = "침묵 (정답)" if got is None else f"★ {got:.3f}m 발행 (오답 — FSM이 믿고 계속 후진한다)"
        print(f"    실제 {truth:.2f}m → {state}")
        if got is not None:
            silent = False

    print("\n[3] 좌우 치우침 — 칸 중앙에서 밀린 정도")
    lat_ok = True
    for shift_px in (0, 40, -40):
        got = find_lateral_error(render(0.60, p, lateral_px=shift_px), p)
        if got is None:
            print(f"    칸 중앙이 {shift_px:+3d}px 밀림 → 못 잼 (실패)")
            lat_ok = False
            continue
        truth = -shift_px * p.m_per_px      # 칸이 오른쪽으로 밀리면 차는 왼쪽으로 치우친 셈
        err = abs(got - truth)
        print(f"    칸 중앙이 {shift_px:+3d}px 밀림 → 치우침 {got * 100:+5.1f}cm "
              f"(정답 {truth * 100:+5.1f}cm, 오차 {err * 100:.1f}cm)")
        if err > 0.03:
            lat_ok = False

    ok = max_err <= 0.03 and silent and lat_ok
    print("\n" + "=" * 66)
    print(f"결과: {'통과' if ok else '실패'}  "
          f"(거리 최대오차 {max_err * 100:.1f}cm / 사각지대 침묵 {'O' if silent else 'X'} / "
          f"치우침 {'O' if lat_ok else 'X'})")
    print("=" * 66)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
