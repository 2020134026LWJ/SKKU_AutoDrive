#!/usr/bin/env python3
"""
버드아이뷰 4점 찍기 — 화면에서 클릭하면 yaml에 넣을 값이 그대로 나온다.

카메라로 도로를 찍으면 비스듬해서(사다리꼴) 곡선을 제대로 못 잰다. 그래서 "위에서
내려다본 것처럼" 펴야 하는데, 컴퓨터는 어디까지가 바닥인지 모른다.
→ **"화면의 이 네 점이 실제로는 직사각형이다"** 를 사람이 알려줘야 한다.

준비: 바닥에 **마스킹 테이프로 직사각형**을 하나 만든다.
      세로 길이를 **줄자로 재둔다** (주차용이면 m_per_px 계산에 쓴다).

네모는 **"보고 싶은 바닥 범위 전체"를 감싸도록 크게** 만들 것.
   네모 **바깥은 펴진 화면에서 잘려나간다.** 작게 만들면:
     - 주차: 뒤 경계선이 네모 밖에 있어서 **아예 안 보인다** → 차가 안 멈춘다
     - 차선: 앞쪽 차선이 잘려서 경로를 못 그린다
   주차용이면 **범퍼 바로 뒤 ~ 1m 이상**까지 덮게 잡아라.

실행:
    python3 scripts/pick_points.py                 # 전방 카메라 (기본 /dev/video0)
    python3 scripts/pick_points.py --cam 1         # 후방 카메라
    python3 scripts/pick_points.py --cam 1 --rear  # 후방 + m_per_px 계산까지
    python3 scripts/pick_points.py --image 사진.png # 저장된 사진으로

쓰는 법:
    1. 화면이 뜨면 테이프 네모의 꼭짓점을 **좌상 → 우상 → 우하 → 좌하** 순서로 클릭
    2. 4개를 다 찍으면 '펴진 화면'이 같이 뜬다
    3. 그 창에서 **네모가 진짜 직사각형으로** 보이면 성공
    4. 터미널에 나온 줄을 config/calibration.yaml 에 복사

    r  = 다시 찍기      s = 결과 출력하고 종료      q = 그냥 종료
"""

import argparse
import sys

import cv2
import numpy as np

W, H = 640, 480
LABELS = ["1) 좌상", "2) 우상", "3) 우하", "4) 좌하"]

points = []


def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
        points.append((float(x), float(y)))
        print(f"  {LABELS[len(points) - 1]} 찍음: ({x}, {y})")


def draw_overlay(frame):
    img = frame.copy()
    for i, (x, y) in enumerate(points):
        cv2.circle(img, (int(x), int(y)), 6, (0, 255, 255), -1)
        cv2.putText(img, str(i + 1), (int(x) + 8, int(y) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    if len(points) >= 2:
        pts = np.int32(points)
        cv2.polylines(img, [pts], len(points) == 4, (0, 255, 255), 2)

    msg = (f"click {len(points)}/4 : {LABELS[len(points)]}" if len(points) < 4
           else "OK  |  s=save  r=redo  q=quit")
    cv2.rectangle(img, (0, 0), (W, 28), (0, 0, 0), -1)
    cv2.putText(img, msg, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    return img


def warped(frame):
    """찍은 4점으로 편 화면. 여기서 네모가 직사각형이면 성공."""
    src = np.float32(points)
    dst = np.float32([(0, 0), (W, 0), (W, H), (0, H)])
    m = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(frame, m, (W, H))


def report(rear: bool):
    flat = ", ".join(f"{v:.1f}" for pt in points for v in pt)
    print("\n" + "=" * 66)
    print("config/calibration.yaml 에 이 줄을 넣으세요")
    print("=" * 66)
    if rear:
        print("\nrear_park_detector_node:")
        print("  ros__parameters:")
        print(f"    src_mat: [{flat}]")
        print("\n[m_per_px 계산]")
        print("  네모가 펴진 화면(480픽셀)을 꽉 채우도록 변환되므로:")
        print("    m_per_px = (네모의 실제 세로 길이 [m]) / 480")
        print("    예: 실제 1.2m 짜리 네모였다면  1.2 / 480 = 0.0025")
        print("\n    m_per_px: <위 계산값>")
        print("\n[확인] 펴진 화면에 **주차칸 뒤 경계선이 보이나요?**")
        print("  안 보이면 네모가 너무 작습니다 — 더 멀리까지 덮게 다시 잡으세요.")
        print("  (네모 바깥은 잘려나갑니다. 안 보이는 선은 차가 못 멈춥니다)")
    else:
        print("\nlane_info_extractor_node:")
        print("  ros__parameters:")
        print(f"    src_mat: [{flat}]")
    print("=" * 66)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--cam', type=int, default=0, help='카메라 번호 (ls /dev/video*)')
    p.add_argument('--image', help='카메라 대신 저장된 사진 사용')
    p.add_argument('--rear', action='store_true', help='후방 카메라(주차용) — m_per_px 계산까지')
    args = p.parse_args()

    if args.image:
        base = cv2.imread(args.image)
        if base is None:
            print(f"사진을 못 읽음: {args.image}")
            return 1
        base = cv2.resize(base, (W, H))
        cap = None
    else:
        cap = cv2.VideoCapture(args.cam)
        if not cap.isOpened():
            print(f"카메라 {args.cam}번을 못 엶. `ls /dev/video*` 로 번호를 확인하세요.")
            return 1
        base = None

    print(__doc__.split('쓰는 법:')[1].split('    r  =')[0])
    print("  (창을 클릭해서 찍으세요. 터미널이 아니라 **창**입니다)\n")

    cv2.namedWindow('camera')
    cv2.setMouseCallback('camera', on_mouse)

    while True:
        if cap is not None:
            ok, frame = cap.read()
            if not ok:
                print("카메라에서 프레임을 못 읽음")
                break
            frame = cv2.resize(frame, (W, H))
        else:
            frame = base.copy()

        cv2.imshow('camera', draw_overlay(frame))

        if len(points) == 4:
            cv2.imshow('pyeojin (warped)', warped(frame))

        key = cv2.waitKey(30) & 0xFF
        if key == ord('r'):
            points.clear()
            cv2.destroyWindow('pyeojin (warped)')
            print("\n다시 찍기\n")
        elif key == ord('s') and len(points) == 4:
            report(args.rear)
            break
        elif key == ord('q'):
            break

    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    sys.exit(main())
