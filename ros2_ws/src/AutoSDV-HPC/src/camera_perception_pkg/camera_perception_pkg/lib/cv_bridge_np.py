"""cv_bridge 대체 — sensor_msgs/Image ↔ numpy 변환 (순수 파이썬).

왜 만들었나
-----------
ROS Jazzy의 `cv_bridge`는 numpy 1.x로 컴파일된 C 확장(`cv_bridge_boost`)에 묶여 있어
numpy 2에서 로드에 실패한다(변환 시 `KeyError: 16`). 그런데 우리 pip 스택
(scipy 1.18 / opencv-python 5 / torch+ultralytics)은 numpy 2를 요구한다.
우리가 cv_bridge에서 쓰는 기능은 **bgr8/mono8 이미지의 바이트↔배열 변환**뿐이라,
그 두 줄을 직접 구현해 의존성을 끊는다.

- ROS 토픽에 흐르는 메시지는 `sensor_msgs/Image` 규격 그대로다. rviz·rosbag 호환 유지.
- 쓰는 쪽 코드는 `from cv_bridge import CvBridge` → `from ..lib.cv_bridge_np import CvBridge`
  한 줄만 바꾸면 된다 (클래스명·메서드 시그니처 동일).

[주의] `np.frombuffer`는 **읽기 전용** 배열을 준다. OpenCV·YOLO가 in-place로 쓰면
`ValueError: assignment destination is read-only`가 나므로 항상 복사본을 돌려준다.
"""

import numpy as np
from sensor_msgs.msg import Image

# 우리 파이프라인이 쓰는 인코딩만. 필요하면 여기 한 줄 추가.
_CHANNELS = {
    "bgr8": 3,
    "rgb8": 3,
    "mono8": 1,
    "8UC3": 3,
    "8UC1": 1,
}


class CvBridgeError(TypeError):
    """cv_bridge.CvBridgeError 자리 대체 (호출부의 except 절 호환)."""


class CvBridge:
    """cv_bridge.CvBridge의 우리 사용분 대체."""

    def imgmsg_to_cv2(self, img_msg: Image, desired_encoding: str = "passthrough") -> np.ndarray:
        """Image 메시지 → numpy 배열 (쓰기 가능한 복사본).

        desired_encoding은 색 변환용이 아니라 '기대하는 인코딩'으로만 취급한다.
        (bgr8↔rgb8 변환이 필요하면 호출부에서 cv2.cvtColor를 쓴다 — 우리 노드는 안 쓴다)
        """
        enc = img_msg.encoding or "bgr8"
        if enc not in _CHANNELS:
            raise CvBridgeError(f"지원하지 않는 인코딩: {enc} (지원: {list(_CHANNELS)})")

        ch = _CHANNELS[enc]
        arr = np.frombuffer(img_msg.data, dtype=np.uint8)
        img = arr.reshape(img_msg.height, img_msg.width, ch)
        if ch == 1:
            img = img[:, :, 0]

        # 읽기 전용 버퍼 → 복사본 (YOLO/OpenCV in-place 쓰기 대비)
        img = img.copy()

        # bgr8 ↔ rgb8만 명시 요청 시 채널 뒤집기 (그 외 요청은 무시 = passthrough)
        if desired_encoding in ("bgr8", "rgb8") and enc in ("bgr8", "rgb8") \
                and desired_encoding != enc:
            img = img[:, :, ::-1].copy()

        return img

    def cv2_to_imgmsg(self, img: np.ndarray, encoding: str = "bgr8") -> Image:
        """numpy 배열 → Image 메시지."""
        if img.dtype != np.uint8:
            raise CvBridgeError(f"uint8만 지원 (받은 dtype: {img.dtype})")

        if img.ndim == 2:
            height, width = img.shape
            ch = 1
            if encoding in ("bgr8", "rgb8", "passthrough"):
                encoding = "mono8"
        elif img.ndim == 3:
            height, width, ch = img.shape
            if encoding in ("passthrough", "mono8"):
                encoding = "bgr8"
        else:
            raise CvBridgeError(f"2D/3D 배열만 지원 (받은 shape: {img.shape})")

        if _CHANNELS.get(encoding) != ch:
            raise CvBridgeError(f"인코딩 {encoding}(채널 {_CHANNELS.get(encoding)})과 "
                                f"배열 채널 수 {ch} 불일치")

        msg = Image()
        msg.height = int(height)
        msg.width = int(width)
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = int(width * ch)
        msg.data = np.ascontiguousarray(img).tobytes()
        return msg
