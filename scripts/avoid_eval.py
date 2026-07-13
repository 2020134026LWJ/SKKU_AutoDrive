#!/usr/bin/env python3
"""
장애물 회피(차선 변경) 판단 채점 — 하드웨어 0, ROS 0.

미션: 차선 주행 중 장애물(차)이 나타나면 옆 차선으로 피했다가 돌아온다.
2~3번 연속, 위치는 랜덤.

가짜 YOLO 결과를 프레임 단위로 먹여서 FSM이 제대로 판단하는지 본다.
실차 영상이 없으니 완벽한 대역은 아니지만, 아래는 지금 확인할 수 있다:

  1) 막히면 피하나            — 내 차선 앞에 차가 오면 옆 차선으로
  2) 지나가면 돌아오나 — 눈앞에서 사라졌다고 바로 돌아오면 옆구리를 긁는다.
                               충분히 지나간 뒤에 돌아와야 한다
  3) 옆 차선 차는 무시하나    — 이미 피한 그 차를 다시 보고 또 피하면 안 된다
  4) 오검출에 안 흔들리나     — 한 프레임 튄 걸로 차선을 바꾸면 위험하다
  5) 연속 장애물을 견디나     — 돌아오는 중에 또 막히면 곧장 다시 피해야 한다

실행:  python3 scripts/avoid_eval.py
"""

import os
import sys
from types import SimpleNamespace

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_LIB = os.path.join(
    _ROOT, 'ros2_ws/src/AutoSDV-HPC/src/decision_making_pkg/decision_making_pkg/lib')
sys.path.insert(0, _LIB)

from avoid_lib import AVOID, FOLLOW, RETURN, AvoidController, AvoidParams  # noqa: E402

P = AvoidParams()
SPEED_PER_FRAME = 0.02   # 한 프레임에 2cm 전진한다고 치자 (30fps에 0.6m/s)


def car(cx, y_max, score=0.9, w=90, h=70):
    """가짜 YOLO detection 하나 (class_name='car')."""
    cy = y_max - h / 2
    return SimpleNamespace(
        class_name='car', score=score,
        bbox=SimpleNamespace(
            center=SimpleNamespace(position=SimpleNamespace(x=cx, y=cy)),
            size=SimpleNamespace(x=w, y=h)))


def lane_mark(cx=320):
    """차선 detection — 회피 로직이 무시해야 하는 것."""
    return SimpleNamespace(
        class_name='lane2', score=0.9,
        bbox=SimpleNamespace(
            center=SimpleNamespace(position=SimpleNamespace(x=cx, y=400)),
            size=SimpleNamespace(x=20, y=200)))


def run(frames, params=P):
    """frames = [detection 리스트, ...] → 각 프레임의 (state, lane) 기록."""
    c = AvoidController(params)
    log = []
    for dets in frames:
        c.add_travel(SPEED_PER_FRAME)
        log.append(c.update(dets))
    return log, c


def check(name, ok, detail=""):
    print(f"    {'통과' if ok else '실패'} — {name}" + (f"  ({detail})" if detail else ""))
    return ok


def main():
    print("=" * 68)
    print("장애물 회피(차선 변경) 판단 채점 — 차 없이")
    print("=" * 68)
    results = []

    # ── 1) 막히면 피한다 ────────────────────────────────────────────────
    print("\n[1] 내 차선 앞에 차가 나타나면 옆 차선으로")
    frames = [[lane_mark()] for _ in range(5)]                    # 깨끗
    frames += [[lane_mark(), car(cx=320, y_max=350)] for _ in range(6)]  # 정면에 차
    log, _ = run(frames)
    results.append(check("회피 시작", log[-1][0] == AVOID and log[-1][1] == P.other_lane,
                         f"마지막 상태 {log[-1]}"))
    # 한 프레임 만에 바꾸면 안 된다 (trigger_frames=3)
    first_avoid = next((i for i, (s, _) in enumerate(log) if s == AVOID), None)
    results.append(check("한 프레임에 즉시 바꾸지 않음",
                         first_avoid is not None and first_avoid >= 5 + P.trigger_frames - 1,
                         f"{first_avoid}번째 프레임에 전환"))

    # ── 2) 지나가면 돌아온다 ────────────────────────────────────────────
    print("\n[2] 장애물을 완전히 지나가면 원래 차선으로 복귀")
    frames = [[lane_mark()] for _ in range(3)]
    frames += [[lane_mark(), car(cx=320, y_max=350)] for _ in range(5)]   # 막힘 → 회피
    frames += [[lane_mark()] for _ in range(40)]                          # 시야에서 사라짐
    log, _ = run(frames)
    results.append(check("복귀함", log[-1][1] == P.home_lane, f"마지막 {log[-1]}"))

    # 사라지자마자 돌아오면 옆구리를 긁는다 — 거리를 채운 뒤에 돌아와야 한다
    avoid_start = next(i for i, (s, _) in enumerate(log) if s == AVOID)
    avoid_end = next(i for i, (s, _) in enumerate(log) if s != AVOID and i > avoid_start)
    frames_in_avoid = avoid_end - avoid_start
    need = P.pass_clearance_m / SPEED_PER_FRAME
    results.append(check("사라지자마자 즉시 복귀하지 않음 (거리 확보)",
                         frames_in_avoid >= need,
                         f"{frames_in_avoid}프레임 유지 (최소 {need:.0f} 필요)"))

    # ── 3) 옆 차선 차는 무시 ────────────────────────────────────────────
    print("\n[3] 옆 차선에 있는 차는 내 길을 막지 않는다")
    frames = [[lane_mark(), car(cx=560, y_max=400)] for _ in range(20)]   # 화면 오른쪽 끝
    log, _ = run(frames)
    results.append(check("옆 차선 차에는 반응 안 함",
                         all(s == FOLLOW for s, _ in log), f"마지막 {log[-1]}"))

    # ── 4) 오검출 방어 ──────────────────────────────────────────────────
    print("\n[4] 한 프레임 튄 오검출로는 차선을 바꾸지 않는다")
    frames = [[lane_mark()] for _ in range(5)]
    frames += [[lane_mark(), car(cx=320, y_max=350)]]     # 딱 1프레임만
    frames += [[lane_mark()] for _ in range(10)]
    log, _ = run(frames)
    results.append(check("1프레임 오검출 무시",
                         all(s == FOLLOW for s, _ in log), f"마지막 {log[-1]}"))

    # ── 5) 연속 장애물 ──────────────────────────────────────────────────
    print("\n[5] 복귀 중에 또 막히면 곧장 다시 피한다 (연속 2~3회)")
    frames = [[lane_mark()] for _ in range(3)]
    frames += [[lane_mark(), car(cx=320, y_max=350)] for _ in range(5)]   # 1번째 장애물
    frames += [[lane_mark()] for _ in range(30)]                          # 지나감 → 복귀
    frames += [[lane_mark(), car(cx=320, y_max=350)] for _ in range(5)]   # 2번째 장애물
    frames += [[lane_mark()] for _ in range(30)]
    frames += [[lane_mark(), car(cx=320, y_max=350)] for _ in range(5)]   # 3번째 장애물
    frames += [[lane_mark()] for _ in range(30)]
    log, _ = run(frames)
    # 회피가 3번 일어났나 (FOLLOW/RETURN -> AVOID 전이 횟수)
    transitions = sum(1 for i in range(1, len(log))
                      if log[i][0] == AVOID and log[i - 1][0] != AVOID)
    results.append(check("장애물 3개를 각각 회피", transitions == 3,
                         f"회피 {transitions}회"))
    results.append(check("마지막엔 원래 차선으로 복귀", log[-1][1] == P.home_lane,
                         f"마지막 {log[-1]}"))

    ok = all(results)
    print("\n" + "=" * 68)
    print(f"결과: {'통과' if ok else '실패'}  ({sum(results)}/{len(results)})")
    print("=" * 68)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
