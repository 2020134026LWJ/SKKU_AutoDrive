# 연결 상태 진단 도구 계획

**작성일:** 2026-03-27 (v3 — 네트워크 구성 반영 + 구현 상세)
**유형:** 독립 Python 스크립트
**실행 위치:** HPC (노트북) 터미널
**목적:** SDV 실험 전 pre-flight checklist

---

## 네트워크 구성

```
HPC (10.0.0.10)
  │
  └─ 10.0.0.x 이더넷 ──┬── ZCU Zone1 (10.0.0.2) ──┬── 11.0.0.x → Camera RPi
                        │                           ├── 12.0.0.x → Camera RPi
                        │                           └── CAN → ECU
                        │
                        ├── ZCU Zone2 (10.0.0.3) ── (동일 구조)
                        ├── ZCU Zone3 (10.0.0.4) ── (동일 구조)
                        └── ZCU Zone4 (10.0.0.5) ── (동일 구조)
```

**HPC에서 접근 가능:** 10.0.0.x 네트워크 + DDS 토픽 (10.0.0.x 경유)
**HPC에서 접근 불가:** 11.0.0.x, 12.0.0.x (ZCU-Camera 전용), CAN (ZCU-ECU 전용)

→ Camera/ECU 상태는 **DDS 데이터 흐름(L3)**으로만 간접 확인 가능

---

## 체크 구조

```
L0 환경     → HPC 자체 ROS2 환경 확인
L1 네트워크 → HPC ↔ ZCU 연결 (10.0.0.x ping)
L3 데이터   → 전체 체인 end-to-end (DDS topic echo)
```

---

## L0 — HPC 환경 자가진단

### 체크 항목

| 체크 | 확인 내용 |
|------|----------|
| ROS2 설치 | `ros2` 명령어가 PATH에 존재하는가 |
| DOMAIN_ID | `ROS_DOMAIN_ID`가 0인가 (ZCU FastDDS는 domain 0) |

### 구현 방법

```python
import shutil
import os

def check_l0():
    results = []

    # ROS2 설치 확인
    ros2_path = shutil.which("ros2")
    results.append(("ROS2 environment", ros2_path is not None))

    # DOMAIN_ID 확인
    domain_id = os.environ.get("ROS_DOMAIN_ID", "0")
    results.append(("ROS_DOMAIN_ID = 0", domain_id == "0"))

    return results
```

**L0 실패 시:** 즉시 종료 (exit 3). "source /opt/ros/humble/setup.bash 실행 필요" 안내.

---

## L1 — 네트워크 (HPC ↔ ZCU ping)

### 체크 항목

| 체크 | 대상 IP | 확인 내용 |
|------|---------|----------|
| ZCU Zone1 | 10.0.0.2 | HPC ↔ ZCU Zone1 이더넷 연결 |
| ZCU Zone2 | 10.0.0.3 | HPC ↔ ZCU Zone2 이더넷 연결 |
| ZCU Zone3 | 10.0.0.4 | HPC ↔ ZCU Zone3 이더넷 연결 |
| ZCU Zone4 | 10.0.0.5 | HPC ↔ ZCU Zone4 이더넷 연결 |

### 구현 방법

```python
import subprocess
import re

def check_ping(ip):
    """
    ping -c 1 -W 1 <ip> 실행
    반환: (성공여부, RTT ms)

    동작 원리:
    - subprocess.run()으로 시스템의 ping 명령어를 실행
    - -c 1: 1회만 전송
    - -W 1: 1초 타임아웃 (응답 없으면 1초 후 실패)
    - returncode 0이면 성공, 그 외 실패
    - stdout에서 "time=1.23 ms" 패턴을 파싱하여 RTT 추출
    """
    try:
        result = subprocess.run(
            ["ping", "-c", "1", "-W", "1", ip],
            capture_output=True,
            text=True,
            timeout=3  # subprocess 자체 타임아웃 (ping이 멈추는 경우 대비)
        )
        if result.returncode == 0:
            # RTT 파싱: "time=1.23 ms" 패턴
            match = re.search(r"time=([\d.]+)\s*ms", result.stdout)
            rtt = match.group(1) if match else "?"
            return True, f"rtt={rtt}ms"
        return False, ""
    except (subprocess.TimeoutExpired, Exception):
        return False, ""
```

**출력 예시:**
```
[L1] Network
  [OK]   ZCU Zone1 (10.0.0.2)  rtt=0.8ms
  [OK]   ZCU Zone2 (10.0.0.3)  rtt=1.2ms
  [FAIL] ZCU Zone3 (10.0.0.4)
  [OK]   ZCU Zone4 (10.0.0.5)  rtt=0.9ms
```

**L1 실패 시:** 해당 Zone의 L3 체크를 SKIP (short-circuit)

---

## L3 — 데이터 흐름 (DDS topic echo)

### 체크 항목

각 Zone마다 (현재 Zone1 기준):

| 체크 | DDS 토픽 | 확인하는 전체 체인 |
|------|---------|-------------------|
| Fisheye | `rt/image_02_raw` | Camera RPi → UDP → ZCU → DDS → HPC |
| Webcam | `rt/image_01_raw` | Camera RPi → UDP → ZCU → DDS → HPC |
| ECU status | `rt/topic_status_signal` | ECU → CAN → ZCU → DDS → HPC |

### 구현 방법

```python
def check_data_flow(topic, timeout=3):
    """
    timeout <N> ros2 topic echo <topic> --once 실행
    반환: 성공여부

    동작 원리:
    - ros2 topic echo --once: 해당 토픽에서 메시지 1개를 수신하면 즉시 종료
    - timeout <N>: N초 내에 메시지가 안 오면 프로세스를 강제 종료 (exit code 124)
    - exit code 0 = 메시지 수신 성공 (해당 체인이 전부 정상)
    - exit code != 0 = 타임아웃 (체인 어딘가 끊김)

    왜 이것으로 Camera/ECU 상태를 알 수 있나:
    - ZCU의 camera_server는 Camera에서 UDP 데이터를 받아야만 DDS로 발행
    - ZCU의 status_publisher는 ECU에서 CAN 0x113을 받아야만 DDS로 발행
    - 따라서 DDS 토픽에 데이터가 있다 = 하위 모듈이 정상 동작 중
    """
    try:
        result = subprocess.run(
            ["timeout", str(timeout), "ros2", "topic", "echo", topic, "--once"],
            capture_output=True,
            timeout=timeout + 2  # subprocess 타임아웃 (timeout 명령어보다 여유 있게)
        )
        return result.returncode == 0
    except (subprocess.TimeoutExpired, Exception):
        return False
```

### short-circuit 로직

```python
def check_zone(zone_name, zone_ip, topics):
    """
    L1이 실패하면 L3를 건너뜀

    동작 원리:
    - ping이 안 되면 DDS도 안 되므로 L3 체크는 무의미
    - SKIP 처리하여 3초 x N개 타임아웃 대기 방지
    """
    ping_ok, rtt = check_ping(zone_ip)
    report(f"{zone_name} ({zone_ip})", ping_ok, rtt)

    if not ping_ok:
        # L3 전부 SKIP
        for name in topics:
            report(f"  {name}", None, "SKIP (network unreachable)")
        return

    # L3 실행
    for name, topic in topics.items():
        ok = check_data_flow(topic)
        report(f"  {name}", ok)
```

---

## 설정값

```python
# 스크립트 상단에 정의 — Zone 추가/변경 시 여기만 수정

ZONES = {
    "Zone1 - Steering": {
        "ip": "10.0.0.2",
        "cameras": {
            "fisheye": "rt/image_02_raw",
            "webcam":  "rt/image_01_raw",
        },
        "ecu_status": "rt/topic_status_signal",
    },
    "Zone2 - Front_Left": {
        "ip": "10.0.0.3",
        "cameras": {
            "fisheye": "rt/image_03_raw",
        },
        "ecu_status": "rt/topic_status_signal",
    },
    "Zone3 - Front_Right": {
        "ip": "10.0.0.4",
        "cameras": {
            "fisheye": "rt/image_04_raw",
        },
        "ecu_status": "rt/topic_status_signal",
    },
    "Zone4 - Rear": {
        "ip": "10.0.0.5",
        "cameras": {
            "fisheye": "rt/image_05_raw",
        },
        "ecu_status": "rt/topic_status_signal",
    },
}

TIMEOUT = 3  # L3 타임아웃 (초)
```

---

## 전체 실행 흐름

```python
def main():
    # 1. L0: 환경 체크
    if not check_l0():
        return 3  # ENV_ERROR

    # 2. Zone별 L1 + L3
    for zone_name, zone_config in ZONES.items():
        check_zone(zone_name, zone_config["ip"], zone_config["data_flow"])

    # 3. 결과 요약
    print_summary()
    return exit_code()
```

---

## 출력 형식

```
AutoSDV Connectivity Diagnostic
================================

[L0] HPC Environment
  [OK] ROS2 environment
  [OK] ROS_DOMAIN_ID = 0

[Zone1 - Steering] ZCU: 10.0.0.2
  +-- HPC <-> ZCU .............. [OK]  rtt=0.8ms
  +-- Camera (fisheye) ......... [OK]  rt/image_02_raw
  +-- Camera (webcam) .......... [FAIL] rt/image_01_raw (timeout 3s)
  +-- ECU (CAN) ................ [OK]  rt/topic_status_signal

[Zone2 - Front_Left] ZCU: 10.0.0.3
  +-- HPC <-> ZCU .............. [OK]  rtt=1.2ms
  +-- Camera (fisheye) ......... [OK]  rt/image_03_raw
  +-- ECU (CAN) ................ [OK]  rt/topic_status_signal

[Zone3 - Front_Right] ZCU: 10.0.0.4
  +-- HPC <-> ZCU .............. [FAIL]
  +-- Camera (fisheye) ......... [SKIP] (ZCU unreachable)
  +-- ECU (CAN) ................ [SKIP] (ZCU unreachable)

[Zone4 - Rear] ZCU: 10.0.0.5
  +-- HPC <-> ZCU .............. [OK]  rtt=0.9ms
  +-- Camera (fisheye) ......... [OK]  rt/image_05_raw
  +-- ECU (CAN) ................ [FAIL] rt/topic_status_signal (timeout 3s)

========================================
SUMMARY
  Zone1 (Steering):    WARN  - Webcam offline
  Zone2 (Front_Left):  OK
  Zone3 (Front_Right): FAIL  - ZCU unreachable
  Zone4 (Rear):        WARN  - ECU offline

  Total: 9/14 passed, 2 skipped, 3 failed
========================================
```

---

## 실패 원인 진단표

| L0 | L1 (ping) | L3 (data) | 진단 |
|----|-----------|-----------|------|
| FAIL | - | - | `source /opt/ros/humble/setup.bash` 실행 필요 |
| OK | FAIL | SKIP | ZCU 전원 또는 이더넷 케이블 확인 |
| OK | OK | 전부 FAIL | ZCU 프로세스 미실행 (`systemctl status zcu`) |
| OK | OK | 카메라만 FAIL | Camera RPi 전원 / cam.py 미실행 |
| OK | OK | ECU만 FAIL | ECU 전원 / CAN 케이블 확인 |
| OK | OK | 전부 OK | 시스템 정상 |

---

## 종료 코드

```
0 = ALL_OK      전부 통과
1 = PARTIAL     일부 실패 (L3)
2 = CRITICAL    L1 실패 (네트워크 연결 불가)
3 = ENV_ERROR   L0 실패 (ROS2 환경 문제)
```

---

## 파일 경로

`Laptop_HPC/diagnostic/diagnostic_check.py`

## 실행 방법

```bash
source /opt/ros/humble/setup.bash
python3 Laptop_HPC/diagnostic/diagnostic_check.py
```

---

## 실행 시간 예측

| 상황 | L0 | L1 | L3 | 총 시간 |
|------|----|----|----|----|
| 전부 정상 | 즉시 | ~1초 x 4 Zone | 각 <1초 | ~8초 |
| 1 Zone 네트워크 실패 | 즉시 | 1초(실패) + 나머지 | SKIP + 나머지 | ~10초 |
| 전부 실패 | 즉시 | 1초 x 4 | 전부 SKIP | ~5초 |
| 네트워크 OK + 데이터 실패 | 즉시 | ~4초 | 3초 x 3 타임아웃 x 4 Zone | 최대 ~40초 |

worst case(모든 Zone의 L3 타임아웃)에서 최대 40초. 실제로는 15초 이내.
