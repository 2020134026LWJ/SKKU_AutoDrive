"""
신호등 인식 로직 (§3.4 대안 A — YOLO 우회, 영상에서 직접 검출)

ROS에 의존하지 않는다 → 노드와 오프라인 테스트(`scripts/test_traffic_light.py`)가
**같은 코드**를 쓴다. 튜닝값을 테스트로 맞추면 그대로 차에서 도는 값이다.

────────────────────────────────────────────────────────────────────────────
왜 '색 비율'이 아니라 'blob'인가 (2026-07-13, 실측으로 갈아엎음)

원래 로직: 화면 위쪽 절반에서 빨강/노랑/초록 픽셀 **비율**을 재고 제일 큰 색을 고름.
→ 녹화 영상(신호등이 **하나도 없는** 실내 로비)에 돌려보니 **71%의 프레임에서
   신호등 색을 뱉었다** (빨강 55% = 주황 기둥, 초록 16% = 잔디천·화분).
   이 값이 motion_planner로 가면 차가 멋대로 서거나 차선추종이 얼어붙는다.

색면(기둥·천·나무)과 점등부(램프)의 차이는 **색이 아니라 형태와 밝기**다:
  - 램프  = 작고 둥글고 **아주 밝다**(V≈255). 어두운 등기구 몸체에 박혀 있다.
  - 색면  = 넓고 형태가 없고 밝기가 평범하다.
→ 밝고 진한 픽셀만 남기고(V·S 하한) **덩어리(contour)를 찾아** 크기·원형도로 거른다.
   "면적이 화면의 절반"인 기둥은 max_area에서 탈락하고, 원형도에서도 탈락한다.

한 프레임만 믿지 않는다 — N프레임 연속 같은 색이어야 확정(`TrafficLightVoter`).
햇빛 반사 한 번에 급정거하지 않기 위함. (라이다 `consec_count=5`와 같은 사고방식)
────────────────────────────────────────────────────────────────────────────
"""

from dataclasses import dataclass

import cv2
import numpy as np


# ── HSV 색 범위 ────────────────────────────────────────────────────────────
# S/V 하한이 핵심 필터다. '빨간 물체'가 아니라 '빨갛게 빛나는 것'을 찾는다.
# (기존 get_traffic_light_color의 H 범위는 재사용, S·V 하한만 크게 올림)
H_RANGES = {
    'Red':    [((0, 10)), ((160, 179))],   # 빨강은 H가 0을 넘어가며 갈라진다 → 두 구간
    'Yellow': [((20, 33))],
    'Green':  [((40, 90))],
}


@dataclass
class TrafficLightParams:
    """전부 ROS 파라미터로 override 가능 (docs/CALIBRATION.md).

    실물 신호등 사진이 나오면 조정할 것은 사실상 min_area/max_area 두 개다
    (신호등이 화면에서 얼마나 크게 보이나 = 거리에 달린 값).
    """
    roi_top: float = 0.0        # 신호등을 찾을 세로 구간 (0=맨위)
    roi_bottom: float = 0.6     # 0.6 = 위쪽 60%. 노면은 볼 이유가 없다.
    sat_min: int = 120          # 채도 하한 — 흰 벽/하늘 반사 배제
    val_min: int = 180          # 명도 하한 점등부는 '빛난다'. 이게 색면을 걸러내는 주력
    min_area: int = 60          # 램프 최소 픽셀 넓이 (너무 작으면 노이즈)
    max_area: int = 8000        # 램프 최대 넓이 기둥·천 같은 대형 색면 배제
    min_circularity: float = 0.55   # 4πA/P² (1.0=완전한 원). 램프는 둥글다
    min_fill_ratio: float = 0.50    # contour 넓이 / 외접사각 넓이. 속 빈 얼룩 배제
    max_aspect: float = 1.6     # 가로/세로 비 상한 램프는 '동그랗다'(비≈1).
                                # 이게 없으면 빨간 자판기의 가로 진열칸(16×6px)이
                                # 램프로 잡힌다 — 꽉 찬 사각형은 원형도(0.62)를
                                # 통과해버리기 때문에 원형도만으론 못 막는다. (실측)
    ring_ratio: float = 0.25    # 등기구를 재는 고리의 두께 (램프 bbox 대비).
                                # 두꺼우면 몸체를 넘어 바깥 배경까지 물어서 진짜
                                # 신호등을 거부하고, 얇으면 오검출을 못 막는다.
    max_surround_ratio: float = 0.55   # 등기구 확인. 램프 둘레(고리)의 밝기가
                                # 램프 자체의 이 배율보다 어두워야 한다.
                                # **점등부는 어두운 몸체에 박혀 있다** — 색·형태가
                                # 아무리 램프 같아도 이건 흉내낼 수 없다.
                                # (빨간 자판기·초록 간판을 걸러낸 최종 필터)
    consec_frames: int = 3      # 같은 색이 N프레임 연속이어야 확정 (반짝임 방어)


def _circularity(contour) -> float:
    perimeter = cv2.arcLength(contour, True)
    if perimeter <= 0:
        return 0.0
    return 4.0 * np.pi * cv2.contourArea(contour) / (perimeter * perimeter)


def _has_dark_housing(v_chan, x, y, w, h, p: TrafficLightParams) -> bool:
    """램프가 '어두운 등기구'에 박혀 있는지 확인 (v_chan = ROI의 HSV V 채널).

    램프 bbox를 절반쯤 키운 사각형에서 bbox 자신을 뺀 **고리**의 평균 밝기를 잰다.
    진짜 신호등이면 그 고리는 검은 몸체다. 빨간 자판기/초록 간판은 둘레도
    밝아서(같은 색이 이어지거나 흰 배경) 여기서 걸린다.

    고리가 화면 밖으로 잘려 표본이 부족하면 **거부한다** — 확인 못 한 건 통과시키지
    않는다(모르면 안전한 쪽). 신호등이 화면 가장자리에 반쯤 걸친 상태면 어차피
    조금 더 다가가서 판단하면 된다.
    """
    ih, iw = v_chan.shape[:2]
    # 고리는 **얇게** 잡는다. 두꺼우면 등기구를 넘어 바깥 배경(하늘·창문)까지 물어서
    # 진짜 신호등을 거부하게 된다 (실측: 고리를 램프 반지름만큼 잡았더니 맨 윗칸
    # 빨간불 검출률이 99.9% → 64%로 떨어졌다).
    mx, my = max(2, int(round(w * p.ring_ratio))), max(2, int(round(h * p.ring_ratio)))
    X0, Y0 = max(0, x - mx), max(0, y - my)
    X1, Y1 = min(iw, x + w + mx), min(ih, y + h + my)

    outer = v_chan[Y0:Y1, X0:X1].astype(np.float32)
    if outer.size == 0:
        return False

    ring_mask = np.ones(outer.shape, dtype=bool)
    ring_mask[y - Y0:y + h - Y0, x - X0:x + w - X0] = False   # 램프 자신은 제외
    ring = outer[ring_mask]
    lamp = v_chan[y:y + h, x:x + w].astype(np.float32)
    if ring.size < 20 or lamp.size == 0:
        return False                              # 표본 부족 → 확인 불가 → 거부

    # 평균이 아니라 **중앙값**. 고리 한쪽 귀퉁이로 밝은 배경이 새어 들어와도
    # 나머지가 검은 몸체면 중앙값은 어둡게 유지된다.
    return float(np.median(ring)) <= p.max_surround_ratio * float(np.median(lamp))


def _color_mask(hsv, color: str, p: TrafficLightParams):
    """그 색으로 '빛나는' 픽셀만 남긴 마스크. (H 범위 + S·V 하한)"""
    mask = None
    for h_lo, h_hi in H_RANGES[color]:
        lo = np.array([h_lo, p.sat_min, p.val_min])
        hi = np.array([h_hi, 255, 255])
        m = cv2.inRange(hsv, lo, hi)
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    # 잡티 제거 → 램프 몸통만 남김
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def find_lamp(frame_bgr, p: TrafficLightParams):
    """한 프레임에서 점등부를 찾는다.

    Returns:
        (color, bbox) — color는 'Red'/'Yellow'/'Green', bbox는 원본 좌표계 (x,y,w,h).
        못 찾으면 (None, None).

    여러 색이 동시에 잡히면 **가장 큰 덩어리**를 택한다 (신호등은 한 번에 하나만 켜진다).
    """
    h_img = frame_bgr.shape[0]
    y0 = int(h_img * p.roi_top)
    y1 = int(h_img * p.roi_bottom)
    roi = frame_bgr[y0:y1, :]
    if roi.size == 0:
        return None, None

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    v_chan = hsv[:, :, 2]

    best = (0.0, None, None)   # (area, color, bbox)
    for color in ('Red', 'Yellow', 'Green'):
        mask = _color_mask(hsv, color, p)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area < p.min_area or area > p.max_area:
                continue                       # 노이즈 / 대형 색면(기둥·천) 탈락
            if _circularity(c) < p.min_circularity:
                continue                       # 램프는 둥글다
            x, y, w, h = cv2.boundingRect(c)
            if w <= 0 or h <= 0:
                continue
            if max(w / h, h / w) > p.max_aspect:
                continue                       # 납작한 띠(자판기 진열칸 등) 탈락
            if area / (w * h) < p.min_fill_ratio:
                continue                       # 속 빈 얼룩 탈락
            if not _has_dark_housing(v_chan, x, y, w, h, p):
                continue                       # 어두운 등기구가 없다 → 램프가 아니다
            if area > best[0]:
                best = (area, color, (x, y + y0, w, h))   # ROI → 원본 좌표

    if best[1] is None:
        return None, None
    return best[1], best[2]


class TrafficLightVoter:
    """N프레임 연속 같은 색일 때만 확정. 한 프레임의 착시로 급정거하지 않기 위함."""

    def __init__(self, consec_frames: int):
        self.consec_frames = consec_frames
        self._candidate = None
        self._count = 0
        self.state = 'None'      # 확정된 색 ('None'이면 신호등 없음)

    def update(self, color) -> str:
        color = color or 'None'
        if color == self._candidate:
            self._count += 1
        else:
            self._candidate = color
            self._count = 1

        if self._count >= self.consec_frames:
            self.state = self._candidate
        return self.state
