# AutoSDV 세션 계획

**작성일:** 2026-03-24 (v3) | **업데이트:** 2026-03-27 (v6 — 코드 재검토 후 새 이슈 반영)

---

## 최종 결과물

1. **빠른 시작** — ZCU/Camera systemd 서비스 최적화
2. **최적화된 ZCU/Camera 코드** — 치명적 버그 수정, 성능 개선
3. **최적화된 ECU 코드** — 워치독, 태스크 관리, 안전장치
4. **연결 상태 진단 도구** — 모듈별 OK/FAIL 체크

## 실행 순서

```
Phase 1: systemd 시작 최적화 + 코드 리뷰/최적화
  1A: systemd 서비스 ✅
  1B: ZCU 소스 코드 ✅ (연계 이슈 2개 남음)
  1C: Camera 모듈 ✅
  1D: ECU 소스 코드 (신규)
Phase 2: 연결 상태 진단 도구
```

---

## Phase 1 진행 상태

### 1A — systemd 서비스 최적화 ✅ 완료

| 서비스 | 변경 내용 |
|--------|----------|
| cam.service | `sleep 3` 제거, `RestartSec=1`, journal 로깅 |
| webcam.service | 신규 생성, cam.service와 동일 패턴 |
| zcu.service | `serial-getty` 제거, 타임아웃/로깅, `network-online.target` 유지 |

---

### 1B — ZCU 소스 코드 최적화 ✅ 완료 (연계 이슈 제외)

**18개 이슈 중 17개 완료, 1개는 ECU 코드로 이관됨.**

| 상태 | 이슈 | 설명 |
|------|------|------|
| ✅ | #18 Zone 설정 외부화 | zcu_config.h, Zone ID 기반 토픽 자동 결정 |
| ✅ | #2 system() sudo 제거 | sudo 제거, CAN_INTERFACE define |
| ✅ | #11 중복 CAN 파일 정리 | status_module 중복 제거 |
| ✅ | #10 strcpy → strncpy | 버퍼 오버플로 방지 |
| ✅ | #1 레이스 컨디션 | 전역 변수 5개 제거, 콜백 내 지역 변수 |
| ✅ | #5 Listener 메모리 누수 | 3개 모듈 listener_ 멤버 관리 + delete |
| ✅ | #4+#15 버퍼 사전 할당 | reserve 재사용, 미사용 buffer 삭제 |
| ✅ | #12 데이터 크기 검증 | 0 또는 >65507 체크 |
| ✅ | #14 이미지 크기 1회 설정 | 루프 전 1회만 |
| ✅ | #3 단일 DDS participant | 4→1 participant, 7개 파일 리팩토링 |
| ✅ | #8 순차 시작 | #3 해결로 자동 개선 |
| ✅ | #6 UDP 타임아웃 | SO_RCVTIMEO 1초 |
| ✅ | #7 CAN 타임아웃 | SO_RCVTIMEO 1초 |
| ✅ | #9 프로토콜 통합 | 1 send = 1 recv |
| ✅ | #16 종료 알림 | sendShutdownMessages 구현 |
| ✅ | #13 조향값 검증 | MIN/MAX 클램핑 |
| ✅ | #17 메인 루프 | condition_variable 대체 |
| → | #19 ECU CAN 타임아웃 | ECU 코드로 이관 (이미 존재, 개선 필요) |

**ECU 연계 이슈 (ZCU 코드 수정 필요):**

| 이슈 | 설명 | 상태 |
|------|------|------|
| Z1 | MIN/MAX_STEERING -15→-7 수정 (ECU 실제 범위와 불일치) | ❌ |
| Z2 | StatusPublisher 종료 메시지 0x113 전송 무의미 (ECU 미수신) | ❌ |

상세: zcu-optimization-plan.md

---

### 1C — Camera 모듈 최적화 ✅ 완료

| 이슈 | 상태 |
|------|------|
| JPEG 재인코딩 루프 제거 (quality=80 고정) | ✅ |
| RGB→BGR 변환 유지 (BGR888 직접 출력 시 색상 뒤바뀜) | ✅ |
| 카메라 초기화 재시도 (wait_for_camera) | ✅ |
| 소켓 타임아웃 | ✅ |
| SIGTERM 핸들링 | ✅ |
| print 문 제거 | ✅ |
| 프로토콜 통합 (ZCU #9 연계) | ✅ |

백업: `cam_bak.py`, `webcam_bak.py`

---

### 1D — ECU 소스 코드 최적화 ❌ 신규

**분석 완료, 구현 대기. 기존 7개 + 코드 재검토 3개 = 10개 이슈.**

| 이슈 | 설명 | 우선순위 | 상태 |
|------|------|----------|------|
| Z1 | ZCU MIN/MAX_STEERING -15→-7 | P0 | ❌ |
| E1 | 워치독 타임아웃 6초→1초 단축 | P0 | ❌ |
| E5 | Task 매초 재생성 → 루프 밖 1회 생성 | P1 | ❌ |
| E-N2 | network.c strlen → dlc 사용 (CAN 바이너리) | P1 | ❌ |
| E3 | 조향 캘리브레이션 실패 처리 | P1 | ❌ |
| E2 | 전역 변수 ISR↔Task 경쟁 보호 | P1 | ❌ |
| Z2 | ZCU 종료 메시지 0x113 제거 | P1 | ❌ |
| E-N1 | ADC ScanStart 중복 호출 제거 | P2 | ❌ |
| E4 | Zone 설정 외부화 (ecu_config.h) | P2 | ❌ |
| E-N3 | 캘리브레이션 첫 샘플 건너뜀 확인 | P2 | ❌ |

상세: ecu-optimization-plan.md

---

## Phase 2: 연결 상태 진단 도구 ❌ 미완

### 2A — 연결 체크 로직
- Camera (RPi) → ZCU: UDP 스트림 수신 여부
- ZCU → HPC: DDS 토픽 발행/구독 확인
- HPC → ZCU: DDS control 토픽 확인
- ZCU → ECU: CAN 0x111/0x123 전송 확인
- ECU → ZCU: CAN 0x113 응답 확인

### 2B — 구현
- 단일 diagnostic_check 스크립트 또는 ROS2 노드
- 모듈별 OK/FAIL, 3초 타임아웃

---

## 전체 진행 요약

| Phase | 항목 | 상태 | 비고 |
|-------|------|------|------|
| 1A | systemd 최적화 | ✅ 완료 | cam/webcam/zcu 서비스 |
| 1B | ZCU 소스 코드 | ✅ 완료 | 17/18 완료, 연계 이슈 2개 (1D에 포함) |
| 1C | Camera 소스 코드 | ✅ 완료 | JPEG, 시그널, 프로토콜 통합 |
| 1B-추가 | ZCU 재검토 이슈 | ❌ 미완 | 새 이슈 6개 (Z-N1~N6) |
| 1D | ECU 소스 코드 | ❌ 미완 | 기존 7개 + 새 3개 = 10개 |
| 2 | 연결 진단 도구 | ✅ 완료 | diagnostic_check.py 구현됨 |

---

## 성공 기준

1. ~~ZCU + Camera 시작 시간 단축~~ → ✅ sleep 제거 + 단일 participant
2. ~~ZCU 치명적 버그 수정~~ → ✅ 레이스 컨디션, 메모리 누수, 버퍼 오버플로
3. 기존 기능에 회귀 없음 → ✅ 현재까지 문제 없음
4. ~~Camera JPEG 인코딩 최적화~~ → ✅ 완료
5. ECU 워치독 6초→1초 + ZCU-ECU 값 범위 일치 → **1D에서 수정**
6. 진단 도구가 모듈 연결 문제를 3초 내 식별 → **Phase 2에서 구현**

---

## 계획 파일 목록

| 파일 | 내용 |
|------|------|
| `.claude/plans/session-plan-ko.md` | 전체 세션 계획 (이 파일) |
| `.claude/plans/zcu-optimization-plan.md` | ZCU 이슈 18개 상세 + 연계 이슈 |
| `.claude/plans/ecu-optimization-plan.md` | ECU 이슈 5개 + ZCU 연계 2개 상세 |
