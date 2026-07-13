#!/usr/bin/env bash
# 주차 FSM 회귀 테스트 — 하드웨어 없이 돈다.
#
#   ./scripts/test_parking.sh            # 3개 시나리오 전부
#   ./scripts/test_parking.sh normal     # 하나만
#
# parking_controller + motion_planner를 띄우고 scripts/parking_sim.py(가짜 뒷벽)를
# 물려서, 주차 시퀀스가 설계대로 흐르는지 + 안전망(라이다 끊김/제한시간)이 실제로
# 발동하는지 확인한다.
#
# [주의] 노드 정리를 pkill로 한다. 이 스크립트를 파일로 둔 이유가 그것 —
# 패턴을 셸 인자로 넘기면 pkill -f 가 자기 자신의 cmdline까지 물어서 스스로 죽는다.
cd "$(dirname "$0")/.."

# [주의] set -u 는 여기서 켜지 않는다. ROS의 setup.bash 들이 미설정 변수를 참조해서,
# set -u 상태로 source 하면 셸이 그 자리에서 죽는다 (출력도 없이).
source scripts/setup_env.sh >/dev/null 2>&1
source ros2_ws/install/setup.bash >/dev/null 2>&1

# CAMERA=1 이면 통합 모드 — 실제 후방 인식 노드(rear_park_detector_node)를 루프에 넣는다.
#   기본(거리 직접 주입) : FSM만 검증. 인식이 틀려도 통과한다.
#   CAMERA=1 (통합)     : 카메라 → 인식 → 판단 → 제어 전체 체인. 진짜 시험.
#     CAMERA=1 ./scripts/test_parking.sh
CAMERA="${CAMERA:-}"

if [ $# -gt 0 ]; then
    SCENARIOS=("$@")
else
    SCENARIOS=(normal occluded never_seen camera_lost timeout)
fi

[ "$CAMERA" = "1" ] && echo "=== 통합 모드: 후방 인식 노드를 루프에 넣는다 ===" || true

cleanup_nodes() {
    pkill -f parking_controller_node >/dev/null 2>&1
    pkill -f motion_planner_node >/dev/null 2>&1
    pkill -f rear_park_detector_node >/dev/null 2>&1
    sleep 1
}

trap cleanup_nodes EXIT

FAILED=0

for scenario in "${SCENARIOS[@]}"; do
    echo "########## $scenario ##########"
    cleanup_nodes

    ros2 run decision_making_pkg parking_controller_node >/tmp/parking_pc.log 2>&1 &
    ros2 run decision_making_pkg motion_planner_node >/tmp/parking_mp.log 2>&1 &

    if [ "$CAMERA" = "1" ]; then
        # 통합 모드: 실제 후방 인식 노드를 루프에 넣는다. 시뮬은 거리 대신 '영상'을 쏜다.
        # 합성 장면을 이미 버드아이뷰로 그리므로 src_mat은 항등변환(실차에선 실측값).
        ros2 run camera_perception_pkg rear_park_detector_node \
            --ros-args -p src_mat:="[0.0,0.0,640.0,0.0,640.0,480.0,0.0,480.0]" \
            >/tmp/parking_rear.log 2>&1 &
    fi
    sleep 2  # DDS discovery

    python3 scripts/parking_sim.py --scenario "$scenario" ${CAMERA:+--camera}
    rc=$?
    [ $rc -ne 0 ] && FAILED=1

    cleanup_nodes
    echo
done

if [ $FAILED -eq 0 ]; then
    echo "전체 통과"
else
    echo "실패한 시나리오가 있다. 노드 로그: /tmp/parking_pc.log, /tmp/parking_mp.log"
fi
exit $FAILED
