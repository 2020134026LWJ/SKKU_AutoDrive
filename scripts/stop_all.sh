#!/usr/bin/env bash
# 떠 있는 ROS 노드를 전부 끈다.
#
# 왜 필요한가: launch로 띄운 노드는 터미널을 닫아도 살아남는 경우가 있다.
# yolov8_node는 매 프레임 추론을 돌리므로 **노트북 배터리를 계속 갉아먹는다.**
# (실제로 몇 시간 방치돼서 배터리가 빨리 닳았다 — 2026-07-13)
#
#   ./scripts/stop_all.sh          # 전부 끄기
#   ./scripts/stop_all.sh --list   # 뭐가 떠 있는지 보기만
#
# [주의] pkill 패턴을 대괄호로 쪼갠다(`ros[2]`). 안 그러면 pkill -f 가
# 자기 자신의 명령줄까지 물어서 스스로 죽는다.

PATTERNS=(
    "launch_pk[g]"
    "camera_perception_pk[g]"
    "decision_making_pk[g]"
    "lidar_perception_pk[g]"
    "serial_communication_pk[g]"
    "parking_si[m]"
    "ros[2] launch"
)

echo "== 떠 있는 노드 =="
FOUND=0
for p in "${PATTERNS[@]}"; do
    while read -r line; do
        [ -n "$line" ] && { echo "  $line"; FOUND=1; }
    done < <(pgrep -af "$p" 2>/dev/null | cut -c1-100)
done
[ $FOUND -eq 0 ] && echo "  (없음)"

if [ "$1" = "--list" ]; then
    exit 0
fi

if [ $FOUND -eq 1 ]; then
    echo
    echo "== 끄는 중 =="
    for p in "${PATTERNS[@]}"; do
        pkill -f "$p" >/dev/null 2>&1
    done
    sleep 2
    for p in "${PATTERNS[@]}"; do
        pkill -9 -f "$p" >/dev/null 2>&1     # 안 죽으면 강제
    done
    sleep 1

    LEFT=0
    for p in "${PATTERNS[@]}"; do
        pgrep -f "$p" >/dev/null 2>&1 && LEFT=1
    done
    if [ $LEFT -eq 0 ]; then
        echo "  전부 종료됨"
    else
        echo "  아직 남음:"
        for p in "${PATTERNS[@]}"; do pgrep -af "$p" 2>/dev/null | cut -c1-100 | sed 's/^/    /'; done
    fi
fi
