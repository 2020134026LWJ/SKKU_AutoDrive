"""
후방 카메라 주차 인식 — 뒤 경계선까지의 거리 + 칸 중앙에서의 좌우 치우침.

ROS에 의존하지 않는다 → 노드와 오프라인 채점(`scripts/rear_park_eval.py`)이 **같은 코드**를
쓴다. 신호등(traffic_light_lib)과 같은 구조다.

────────────────────────────────────────────────────────────────────────────
왜 이 노드가 필요한가

원래 주차는 후방 **라이다**로 '뒷벽'까지의 거리를 재서 멈췄다. 그런데 실제 주차장엔
**뒷벽이 없고 양옆에 차만 있다**(2026-07-13 확인). 벽이 없으면 라이다는 멈출 근거를
못 준다 → 차가 주차칸을 뚫고 나간다.

그래서 멈추는 근거를 **주차칸 뒤 경계선(가로 흰선)** 으로 바꾼다. 그 선까지의 거리를
후방 카메라로 재서 parking_rear_distance로 발행하면, 주차 FSM은 출처를 모른 채
기존 로직(비례감속 → 정지) 그대로 돈다.

핵심 제약 두 가지 (주차 FSM이 이걸 전제로 짜여 있다):

  1) **옆차가 뒷선의 좌우를 가린다** → 화면 전체가 아니라 **중앙 띠**만 본다.
     우리 차 바로 뒤는 마지막까지 열려 있다.
  2) **다 들어가면 뒷선이 범퍼 밑 사각지대로 사라진다** → 그때 거리를 **발행하지 않는다.**
     억지로 추정해서 내보내면 FSM이 그 거짓값을 믿고 계속 후진한다.
     "안 보이면 침묵한다"가 여기서의 정직함이고, FSM은 침묵을 '다 들어왔다'로 해석한다.
────────────────────────────────────────────────────────────────────────────
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class RearParkParams:
    """전부 ROS 파라미터로 override 가능 (docs/CALIBRATION.md '주차' 절).

    ★실측이 꼭 필요한 것은 두 개다:
      - src_mat  : 버드아이뷰 4점 (후방 카메라로 바닥 사각형을 찍어 잡는다)
      - m_per_px : 버드아이뷰 세로 1픽셀이 실제 몇 m인가
    나머지는 '흰 선의 생김새'라 웬만하면 안 건드려도 된다.
    """
    # 버드아이뷰 변환 (기본값은 전방 차선용 값을 옮겨온 것 — 후방용으로 반드시 재측정)
    src_mat: List[Tuple[int, int]] = field(
        default_factory=lambda: [(160, 180), (480, 180), (640, 400), (0, 400)])
    dst_mat: List[Tuple[int, int]] = field(
        default_factory=lambda: [(0, 0), (640, 0), (640, 480), (0, 480)])

    m_per_px: float = 0.0022     # [m/px] ★실측. BEV 세로 1px이 실제 몇 m인가
    cam_offset_m: float = 0.10   # [m] 카메라 렌즈 ~ 뒤범퍼 끝 거리. 거리를 '범퍼 기준'으로 보정

    center_band: float = 0.34    # 화면 폭 대비 중앙 띠 비율 ★옆차 가림 회피의 핵심
    white_min: int = 170         # 흰 선으로 볼 최소 밝기 (V)
    sat_max: int = 90            # 흰색은 채도가 낮다 — 색 있는 바닥/차체 배제

    min_line_px: int = 40        # 가로선으로 인정할 최소 흰 픽셀 수 (중앙 띠 한 행 기준)
    min_line_ratio: float = 0.45  # 중앙 띠 폭 대비 흰 픽셀 비율 하한. 얼룩/반사 배제

    # 좌우 치우침 (옆차 기준). 옆차는 바닥이 아니라 '서 있는 물체'라 사각지대에 안 들어간다.
    #
    # [중요] '어둡다'를 절대 밝기로 정하지 않는다. 조명이 바뀌면 그대로 깨진다 —
    # 처음에 V<110으로 뒀더니 바닥(V=96)까지 차로 잡혀서 화면 전체가 '옆차'가 됐다.
    # 옆차의 조건은 '어둡다'가 아니라 **'바닥보다 어둡다'** 이다.
    side_darker_than: float = 0.75   # 바닥 밝기의 이 배율보다 어두운 열 = 옆차
    min_side_cols: int = 20          # 옆차로 인정할 최소 폭 [px]. 이보다 얇으면 얼룩
    min_slot_cols: int = 60          # 두 차 사이에 이만큼은 열려 있어야 '칸'이다


def _bev(frame_bgr, p: RearParkParams):
    src = np.float32(p.src_mat)
    dst = np.float32(p.dst_mat)
    m = cv2.getPerspectiveTransform(src, dst)
    h, w = frame_bgr.shape[:2]
    return cv2.warpPerspective(frame_bgr, m, (w, h))


def _white_mask(bev, p: RearParkParams):
    """흰 선만 남긴다. 밝고(V 높음) 채도 낮은(S 낮음) 픽셀."""
    hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
    lo = np.array([0, 0, p.white_min])
    hi = np.array([179, p.sat_max, 255])
    mask = cv2.inRange(hsv, lo, hi)
    kernel = np.ones((3, 3), np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)


def find_rear_line(frame_bgr, p: RearParkParams) -> Optional[float]:
    """뒤 경계선까지의 거리 [m]. 못 찾으면 None (= 침묵. 억지로 추정하지 않는다).

    버드아이뷰에서 **중앙 띠**만 훑어, 가로로 길게 이어진 흰 행을 찾는다.
    BEV는 위에서 내려다본 그림이라 '아래쪽 = 차에 가까움'이다. 따라서 가장 아래에서
    발견된 가로선이 우리가 다가가는 그 선이다.
    """
    bev = _bev(frame_bgr, p)
    h, w = bev.shape[:2]
    mask = _white_mask(bev, p)

    half = int(w * p.center_band / 2)
    cx = w // 2
    band = mask[:, max(0, cx - half):min(w, cx + half)]
    band_w = band.shape[1]
    if band_w == 0:
        return None

    counts = (band > 0).sum(axis=1)          # 행마다 흰 픽셀 수
    need = max(p.min_line_px, int(band_w * p.min_line_ratio))
    rows = np.where(counts >= need)[0]
    if rows.size == 0:
        return None                          # 안 보인다 → 침묵

    # BEV 아래쪽이 차에 가깝다 → 가장 아래 행이 우리가 다가가는 선.
    # (연속된 행 덩어리의 아래쪽 끝을 쓴다 — 선은 두께가 있다)
    row = int(rows.max())

    px_to_bottom = (h - 1) - row
    return px_to_bottom * p.m_per_px + p.cam_offset_m


def find_lateral_error(frame_bgr, p: RearParkParams) -> Optional[float]:
    """칸 중앙 대비 좌우 치우침 [m]. 양수 = 차가 오른쪽으로 치우침. 못 재면 None.

    양옆 차를 기준으로 잡는다. 옆차는 **서 있는 물체**라 뒷선과 달리 사각지대에
    들어가지 않는다 — 즉 마지막까지 정렬 근거로 쓸 수 있다.
    두 대가 다 안 보이면(한쪽만 보이거나 아무것도 없으면) None을 낸다. 한쪽만으로
    중앙을 정하면 틀린 중앙을 확신하게 되므로, 모르면 모른다고 하는 게 낫다.
    """
    bev = _bev(frame_bgr, p)
    h, w = bev.shape[:2]
    hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
    v = hsv[:, :, 2]

    # 바닥 밝기를 먼저 잡는다. 중앙 띠는 '칸 안'이라 바닥이 보장된다.
    half = max(1, int(w * p.center_band / 2))
    cx = w // 2
    floor_v = float(np.median(v[:, max(0, cx - half):min(w, cx + half)]))
    if floor_v <= 1.0:
        return None

    # 열마다 '이 열은 차인가'(= 바닥보다 어두운가)를 판정 →
    # 좌우 끝에서 **이어진** 어두운 덩어리를 찾는다.
    #
    # [주의] 예전엔 화면 좌/우 가장자리 30% 띠 안에서만 찾았는데, 칸이 옆으로 밀리면
    # 옆차의 안쪽 가장자리가 그 띠 밖으로 나가서 **띠 경계를 차 가장자리로 착각**했다
    # (치우침이 항상 0으로 나옴). 띠를 없애고 가장자리에서 이어진 길이를 재면 그 문제가 사라진다.
    col_v = v.mean(axis=0)
    dark_col = col_v < floor_v * p.side_darker_than

    left_w = 0
    while left_w < w and dark_col[left_w]:
        left_w += 1
    right_w = 0
    while right_w < w and dark_col[w - 1 - right_w]:
        right_w += 1

    if left_w < p.min_side_cols or right_w < p.min_side_cols:
        return None                 # 양옆 차를 다 못 봤다 → 모른다.
                                    # 한쪽만으로 중앙을 정하면 '틀린 중앙'을 확신하게 된다.
    if (w - left_w - right_w) < p.min_slot_cols:
        return None                 # 두 덩어리가 화면을 거의 다 덮었다 = 칸이 안 보인다

    left_edge = left_w - 1          # 왼쪽 차의 안쪽 면
    right_edge = w - right_w        # 오른쪽 차의 안쪽 면

    slot_center = (left_edge + right_edge) / 2.0
    car_center = w / 2.0
    return float((car_center - slot_center) * p.m_per_px)
