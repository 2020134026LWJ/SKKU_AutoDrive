"""
장애물 회피 판단 로직 (ROS 비의존).

노드와 오프라인 테스트(`scripts/avoid_eval.py`)가 **같은 코드**를 쓴다.
주차(rear_park_lib) / 신호등(traffic_light_lib)과 같은 구조.

────────────────────────────────────────────────────────────────────────────
미션: 차선 주행 중 장애물(차 형태)이 나타나면 **옆 차선으로 피했다가 돌아온다.**
2~3번 연속으로, 위치는 랜덤.

핵심 설계 — **핸들을 억지로 꺾지 않는다.**

lane_info_extractor는 원래 'lane2' 하나만 따라가도록 하드코딩돼 있었다. 즉
**"어느 차선을 따라갈지"를 바꾸는 것만으로 차선 변경이 된다.** 그러면 기존
경로계획(path_planner)이 알아서 부드러운 경로를 그리고 motion_planner가 그걸
따라간다. 회피 로직이 조향값을 직접 만들면 그 순간 차선 추종과 싸우게 된다.

  YOLO(car) ─→ [이 로직] ─→ target_lane ─→ lane_info_extractor ─→ path_planner ─→ 제어

그래서 이 파일이 하는 일은 딱 하나: **"지금 어느 차선을 따라가야 하나"를 정하는 것.**
────────────────────────────────────────────────────────────────────────────
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple

# FSM 상태
FOLLOW = "FOLLOW"      # 기본 차선 주행
AVOID = "AVOID"        # 옆 차선으로 피해 있는 중
RETURN = "RETURN"      # 원래 차선으로 복귀하는 중


@dataclass
class AvoidParams:
    """전부 ROS 파라미터로 override 가능 (docs/CALIBRATION.md)."""

    # 차선 이름 — YOLO 모델이 아는 클래스 (best_urp.pt: lane1, lane2)
    # [주의] lane1/lane2 중 어느 게 좌/우인지는 **모델 학습 때 정해진 것이라 코드로는 알 수 없다.**
    #   실제 영상에서 눈으로 확인해서 맞출 것 (docs/CALIBRATION.md).
    home_lane: str = "lane2"     # 기본으로 달리는 차선
    other_lane: str = "lane1"    # 피할 때 옮겨갈 차선

    obstacle_class: str = "car"  # 장애물 클래스 (모델에 이미 있음)

    img_width: int = 640
    img_height: int = 480

    # "내 앞을 막고 있나" 판정
    #   가까울수록 bbox 아래변(y_max)이 화면 아래로 내려온다.
    trigger_y: int = 300         # bbox 아래변이 이 아래로 오면 '가깝다' 실측
    min_score: float = 0.4       # YOLO 신뢰도 하한
    lane_half_width: int = 120   # 내 차선의 반폭 [px] — bbox 중심이 이 안이면 '내 차선' 실측

    # 복귀 판정
    clear_frames: int = 5        # 장애물이 이만큼 연속으로 안 보이면 '지나갔다'
    # 복귀 전 최소 전진 거리 — 장애물 옆을 완전히 지나기 전에 돌아오면 **긁는다.**
    # 오도메트리가 없으니 속도 적분으로 추정한다 (주차 blind_travel과 같은 방식).
    pass_clearance_m: float = 0.40   # 실측 (차 길이 + 여유)

    # 오검출 방어 — 한 프레임 보고 차선을 바꾸지 않는다
    trigger_frames: int = 3      # 이만큼 연속으로 보여야 회피 시작


@dataclass
class Obstacle:
    """YOLO detection에서 우리가 필요한 것만 추린 것."""
    cx: float          # bbox 중심 x
    cy: float          # bbox 중심 y
    w: float
    h: float
    score: float

    @property
    def y_max(self) -> float:
        return self.cy + self.h / 2


def obstacles_from_detections(detections, p: AvoidParams) -> List[Obstacle]:
    """DetectionArray → 장애물 목록 (obstacle_class 만, 신뢰도 하한 통과분).

    detections는 ROS 메시지든 테스트용 가짜 객체든 상관없다 — 같은 속성만 있으면 된다.
    """
    out = []
    for d in detections:
        if d.class_name != p.obstacle_class:
            continue
        if d.score < p.min_score:
            continue
        out.append(Obstacle(
            cx=d.bbox.center.position.x,
            cy=d.bbox.center.position.y,
            w=d.bbox.size.x,
            h=d.bbox.size.y,
            score=d.score,
        ))
    return out


def blocking_obstacle(obstacles: List[Obstacle], p: AvoidParams) -> Optional[Obstacle]:
    """'내 차선을 막고 있는' 장애물. 없으면 None.

    두 조건을 다 만족해야 한다:
      1) 내 차선 안에 있다  — bbox 중심이 화면 중앙에서 lane_half_width 안쪽
      2) 가깝다             — bbox 아래변이 trigger_y 아래로 내려왔다

    옆 차선에 서 있는 차(이미 피한 뒤의 그 차 포함)는 1)에서 걸러진다.
    멀리 있는 차는 2)에서 걸러진다 — 아직 피할 때가 아니다.
    """
    center = p.img_width / 2
    best = None
    for o in obstacles:
        if abs(o.cx - center) > p.lane_half_width:
            continue                      # 옆 차선 차 — 내 길을 막지 않는다
        if o.y_max < p.trigger_y:
            continue                      # 아직 멀다
        if best is None or o.y_max > best.y_max:
            best = o                      # 여럿이면 제일 가까운 것
    return best


class AvoidController:
    """차선 변경 FSM.

    상태는 셋뿐이다:
      FOLLOW — 기본 차선(home_lane) 주행. 앞이 막히면 AVOID로.
      AVOID  — 옆 차선(other_lane) 주행. 장애물을 완전히 지나가면 RETURN으로.
      RETURN — 기본 차선으로 복귀 중. 복귀가 끝나면 FOLLOW로.
               (복귀 중에 또 막히면 곧장 AVOID로 — 연속 장애물 대응)
    """

    def __init__(self, params: AvoidParams):
        self.p = params
        self.state = FOLLOW
        self.target_lane = params.home_lane
        self._trigger_count = 0     # 장애물이 연속으로 보인 프레임 수
        self._clear_count = 0       # 장애물이 연속으로 안 보인 프레임 수
        self._travel_since_avoid = 0.0   # AVOID 진입 후 전진한 거리 [m]

    def add_travel(self, meters: float) -> None:
        """전진 거리 누적 (속도 적분). 노드가 매 주기 호출한다."""
        if self.state == AVOID:
            self._travel_since_avoid += meters

    def update(self, detections) -> Tuple[str, str]:
        """한 프레임 처리. (state, target_lane) 반환."""
        obstacles = obstacles_from_detections(detections, self.p)
        blocker = blocking_obstacle(obstacles, self.p)

        if blocker is not None:
            self._trigger_count += 1
            self._clear_count = 0
        else:
            self._clear_count += 1
            self._trigger_count = 0

        if self.state == FOLLOW:
            # 한 프레임 보고 차선을 바꾸지 않는다 (오검출 방어)
            if self._trigger_count >= self.p.trigger_frames:
                self.state = AVOID
                self.target_lane = self.p.other_lane
                self._travel_since_avoid = 0.0

        elif self.state == AVOID:
            # 장애물이 시야에서 사라졌다고 바로 돌아오면 **차 옆구리를 긁는다.**
            # 눈앞에서 사라진 것과 완전히 지나친 것은 다르다 → 거리로도 확인한다.
            passed = (self._clear_count >= self.p.clear_frames
                      and self._travel_since_avoid >= self.p.pass_clearance_m)
            if passed:
                self.state = RETURN
                self.target_lane = self.p.home_lane

        elif self.state == RETURN:
            # 복귀 중에 또 막히면(연속 장애물) 곧장 다시 피한다.
            if self._trigger_count >= self.p.trigger_frames:
                self.state = AVOID
                self.target_lane = self.p.other_lane
                self._travel_since_avoid = 0.0
            elif self._clear_count >= self.p.clear_frames:
                self.state = FOLLOW

        return self.state, self.target_lane
