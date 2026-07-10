# ZCU 소스 코드 최적화 계획

**작성일:** 2026-03-26 | **업데이트:** 2026-03-27 (v4 — 코드 재검토 후 새 이슈 추가)
**대상:** S32G3_ZCU 전체 소스 코드

---

## 진행 상태 요약

**18개 이슈 중 17개 완료. #19(ECU 타임아웃)는 ECU 코드로 이관.**

| 순서 | 이슈 | 우선순위 | 상태 |
|------|------|----------|------|
| 1 | #18 Zone 설정 외부화 | P0 | ✅ zcu_config.h, #if 분기, cmake 빌드 옵션 |
| 2 | #2 system() sudo 제거 | P0 | ✅ sudo 제거, CAN_INTERFACE define |
| 3 | #11 중복 CAN 파일 정리 | P1 | ✅ status_module 중복 _bak으로 이동 |
| 4 | #10 strcpy → strncpy | P1 | ✅ IFNAMSIZ 사용 |
| 5 | #1 레이스 컨디션 | P0 | ✅ 전역 변수 5개 제거, 콜백 내 지역 변수 |
| 6 | #5 Listener 메모리 누수 | P0 | ✅ 3개 모듈 listener_ 멤버 + delete |
| 7 | #4+#15 버퍼 사전 할당 | P0 | ✅ reserve(65507) 재사용, buffer 삭제 |
| 8 | #12 데이터 크기 검증 | P1 | ✅ 0 또는 >65507 체크 |
| 9 | #14 이미지 크기 1회 설정 | P2 | ✅ 루프 전 1회만 |
| 10 | #3 단일 DDS participant | P0 | ✅ 4→1 participant, 7개 파일 리팩토링 |
| 11 | #6 UDP 타임아웃 | P1 | ✅ SO_RCVTIMEO 1초 |
| 12 | #7 CAN 타임아웃 | P1 | ✅ SO_RCVTIMEO 1초 |
| 13 | #9 프로토콜 통합 | P1 | ✅ 1 send = 1 recv (cam.py + webcam.py + camera_server) |
| 14 | #16 종료 알림 | P1 | ✅ sendShutdownMessages 구현 (CAN_STATUS_ID 0xFF) |
| 15 | #13 조향값 검증 | P2 | ✅ MIN/MAX_STEERING 클램핑 |
| 16 | #17 메인 루프 폴링 | P2 | ✅ condition_variable 대체 |
| 17 | #8 순차 시작 | P1 | ✅ #3 해결로 자동 개선 |
| - | #19 ECU CAN 타임아웃 | P0 | → ECU 계획으로 이관 |

---

## ECU 연계 이슈 (ZCU 코드 수정 필요)

| 이슈 | 설명 | 상태 |
|------|------|------|
| Z1 | MIN/MAX_STEERING -15→-7 수정 (ECU 실제 범위와 불일치) | ❌ |
| Z2 | StatusPublisher 종료 메시지 0x113 전송 무의미 (ECU 미수신) | ❌ |

상세: ecu-optimization-plan.md 참조

---

## 수정된 파일 목록

| 파일 | 주요 변경 |
|------|----------|
| `include/zcu_config.h` | **신규** — Zone 설정 집중, #if 분기 |
| `src/main.cpp` | DDS participant 생성, condition_variable, zcu_config.h 적용 |
| `src/camera_module/camera_server.cpp` | 버퍼 reserve, 프로토콜 통합, UDP 타임아웃, listener 관리 |
| `src/camera_module/camera_server.hpp` | participant 포인터 + listener_ 멤버 추가 |
| `src/control_module/control_subscriber.cpp` | 전역 변수 제거, 조향 클램핑, CAN ID define |
| `src/control_module/control_subscriber.hpp` | participant 포인터 추가 |
| `src/control_module/s32g3_skku_can_setting.c` | sudo 제거, strncpy, zcu_config.h |
| `src/status_module/status_publisher.cpp` | CAN 타임아웃, 종료 알림, listener 관리 |
| `src/status_module/status_publisher.hpp` | participant 포인터 + listener_ 멤버 추가 |
| `CMakeLists.txt` | ZCU_ZONE_ID 빌드 옵션 |

백업: 모든 원본에 `_bak` 접미사 파일 존재.

---

## 코드 재검토 후 추가 발견 이슈 (2026-03-27)

| # | 심각도 | 문제 | 파일 | 상태 |
|---|--------|------|------|------|
| Z-N1 | **높음** | ControlSubscriber 생성자에서 listener_, subscriber_, reader_, topic_ 미초기화 → stop() 시 dangling pointer 위험 | control_subscriber.cpp | ❌ |
| Z-N2 | **높음** | StatusPublisher 동일 미초기화 (listener_, publisher_, writer_, topic_) | status_publisher.cpp | ❌ |
| Z-N3 | 중간 | ioctl() 반환값 체크 없음 (CAN 인터페이스 설정 실패 시 계속 진행) | s32g3_skku_can_setting.c | ❌ |
| Z-N4 | 중간 | ip link set down 반환값 체크 없음 | s32g3_skku_can_setting.c | ❌ |
| Z-N5 | 낮음 | eth_socket 전역 변수 선언되었으나 미사용 | s32g3_skku_can_setting.c | ❌ |
| Z-N6 | 낮음 | CAN 메시지 매직 넘버(+7, 0xFF) 미문서화 | control_subscriber.cpp | ❌ |

### Z-N1, Z-N2 상세 (미초기화 멤버)

CameraServer는 올바르게 초기화:
```cpp
CameraServer::CameraServer(...) : ..., listener_(nullptr), ... {}
```

ControlSubscriber/StatusPublisher는 미초기화:
```cpp
ControlSubscriber::ControlSubscriber(DomainParticipant* participant)
    : participant_(participant), running_(false) {
    // listener_, subscriber_, reader_, topic_ 미초기화!
}
```

setupDDS()에서 예외 발생 시 stop()의 nullptr 체크가 무의미 → 해결: 이니셜라이저 리스트에 nullptr 추가
