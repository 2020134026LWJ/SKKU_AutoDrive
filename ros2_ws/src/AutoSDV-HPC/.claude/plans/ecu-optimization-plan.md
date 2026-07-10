# ECU (RA6M5) 소스 코드 최적화 계획

**작성일:** 2026-03-26
**대상:** RA6M5_ECU/adc_rtos 전체 소스 코드
**빌드 환경:** Renesas e2 studio / FSP (FreeRTOS)
**참조:** ZCU 최적화 계획 `.claude/zcu-optimization-plan.md`

---

## 아키텍처 요약

```
FreeRTOS (RA6M5)
├─ Blinky Thread (메인) — HW 초기화, LED 토글, 1초 루프
├─ PotentiometerTask (Steering_Zone 전용, 50ms) — ADC → 조향 모터 제어
├─ SteeringStatusTask (Steering_Zone 전용, 300ms) — CAN 0x113 현재 각도 송신
├─ canfd0_callback (ISR) — CAN 수신: 0x111(조향), 0x123(속도)
└─ R_AGT0_Interrupt (ISR) — 2초 주기 워치독
```

**하드웨어 Zone별 역할:**
- Steering_Zone: 조향 모터 + 포텐셔미터 + CAN 0x111 수신/0x113 송신
- Front_Left / Front_Right / Rear_Left / Rear_Right: 구동 모터 + CAN 0x123 수신

---

## 파일 구조

```
RA6M5_ECU/adc_rtos/src/
├── blinky_thread_entry.c  ← 메인 태스크, CAN 콜백, 워치독, 조향 제어
├── hardware.c             ← Motor_Control, ADC, PWM
├── hardware.h             ← 하드웨어 추상화, enum
├── network.c              ← CANFD 초기화
└── motor_test.c           ← 테스트 모드 (비활성)
```

---

## 이슈 목록

### E1 — 워치독 타임아웃 6초 → 1초로 단축 [P0 — 안전]

**파일:** `blinky_thread_entry.c:296-307`

**문제:**
```c
void R_AGT0_Interrupt(timer_callback_args_t *p_args) {
    if(motor_stop == 0) { motor_stop++; }      // 2초 후
    else if(motor_stop == 1) { motor_stop++; }  // 4초 후
    else if(motor_stop == 2) { Motor_Stop(); }   // 6초 후 정지
}
```
Timer0 주기 2초 x 3단계 = **6초** 동안 모터가 무제어 상태로 동작. 1/5 스케일 차량에 위험.

**해결:**
- Timer0 period를 500ms로 변경 (hal_data.c의 AGT 설정)
- 인터럽트 로직 간소화: 1회 미수신(500ms) 시 즉시 정지
```c
void R_AGT0_Interrupt(timer_callback_args_t *p_args) {
    if (motor_stop >= 1) {
        Motor_Stop();
    } else {
        motor_stop++;
    }
}
```
- CAN 수신 콜백에서 `motor_stop = 0` 리셋은 유지

---

### E2 — 전역 변수 ISR↔Task 경쟁 [P1]

**파일:** `blinky_thread_entry.c`

**문제:**
- `motor_stop` (volatile int): ISR(canfd0_callback)에서 리셋, ISR(R_AGT0_Interrupt)에서 증가, Task에서 읽기
- `g_can_steering_rx_frame`, `g_can_rx_frame`: ISR에서 쓰기, Task에서 읽기
- `angle`: Task에서 쓰기/읽기

단일 코어이므로 ISR은 원자적이지만, Task 간 공유는 선점에 의해 데이터 손상 가능.

**해결:**
- `motor_stop`은 `volatile`로 이미 선언되어 있어 OK
- CAN 프레임 접근 시 인터럽트 비활성화 또는 FreeRTOS 큐 사용:
```c
taskENTER_CRITICAL();
local_frame = g_can_steering_rx_frame;
taskEXIT_CRITICAL();
```

---

### E3 — 조향 캘리브레이션 실패 처리 없음 [P1]

**파일:** `blinky_thread_entry.c:145-178`

**문제:**
캘리브레이션 중 ADC 읽기 실패 시 예외 처리 없음. 좌/우 최대값이 동일하면 나눗셈에서 0으로 나누기 가능.
또한 캘리브레이션이 완료되지 않으면 무한 루프.

**해결:**
- ADC 읽기 실패 시 기본값 사용
- 좌/우 값 sanity check: `if (abs(left - right) < threshold) { use_defaults(); }`
- 캘리브레이션 전체에 타임아웃 추가 (예: 10초)

---

### E4 — CAN 바이트 매핑 하드코딩 [P2]

**파일:** `blinky_thread_entry.c:323-343`

**문제:**
```c
if(Hardware == Front_Left) Motor_Control(data[0], data[3]);
if(Hardware == Front_Right) Motor_Control(data[4], data[7]);
```
Zone별 CAN 바이트 매핑이 switch-case로 하드코딩.

**해결:**
설정 헤더 `ecu_config.h`에 Zone별 바이트 오프셋 정의:
```c
#define MOTOR_DIR_OFFSET  0  // 또는 4
#define MOTOR_SPD_OFFSET  3  // 또는 7
```
ZCU의 zcu_config.h와 동일 패턴.

---

### E5 — Task 매초 재생성 (메모리 누수) [P1]

**파일:** `blinky_thread_entry.c:225-229`

**문제:**
```c
while(1) {
    xTaskCreate(PotentiometerTask, ...);    // 매초 새 태스크 생성
    xTaskCreate(SteeringStatusTask, ...);   // 매초 새 태스크 생성
    vTaskDelay(1000);
}
```
태스크가 완료되지 않으면 매초 새로 생성 → FreeRTOS 힙 고갈 → 크래시.

**해결:**
태스크 생성을 루프 밖으로 이동:
```c
if (Hardware == Steering_Zone) {
    xTaskCreate(PotentiometerTask, ...);
    xTaskCreate(SteeringStatusTask, ...);
}
while(1) {
    // LED 토글만
    vTaskDelay(1000);
}
```

---

## ZCU 연계 이슈 (ZCU 코드도 수정 필요)

### Z1 — 조향 클램핑 범위 불일치 [P0 — ZCU 수정]

**파일:** `S32G3_ZCU/include/zcu_config.h`

**문제:**
ZCU: `MIN_STEERING=-15, MAX_STEERING=15`
ECU: angle 범위 = `-7 ~ +7`, CAN data[3] = `angle + 7` = `0 ~ 14`

ZCU에서 15를 보내면 → `(15+7) & 0xFF = 22` → ECU에서 14 초과 = 예측 불가.

**수정:** `zcu_config.h`에서:
```c
#define MIN_STEERING  (-7)
#define MAX_STEERING  (7)
```

### Z2 — StatusPublisher 종료 메시지 경로 오류 [P1 — ZCU 수정]

**파일:** `S32G3_ZCU/src/status_module/status_publisher.cpp`

**문제:**
`sendShutdownMessages()`가 CAN 0x113으로 종료 메시지 전송.
하지만 ECU는 0x113을 **수신하지 않음** (송신만 함).

ECU가 수신하는 ID: 0x111(조향), 0x123(속도).
`ControlSubscriber::sendShutdownMessages()`가 이미 0x111/0x123으로 올바르게 전송 중.

**수정:** `StatusPublisher::sendShutdownMessages()`의 CAN 전송 제거:
```cpp
void StatusPublisher::sendShutdownMessages() {
    // ECU는 0x113을 수신하지 않음
    // 종료 시 ControlSubscriber가 0x111/0x123으로 정지 명령 전송
}
```

---

## 코드 재검토 후 추가 발견 이슈 (2026-03-27)

| # | 심각도 | 문제 | 파일 | 상태 |
|---|--------|------|------|------|
| E-N1 | 중간 | R_ADC_ScanStart 중복 호출 (ADC_Setting + Potentiometer_Read 양쪽) | hardware.c | ❌ |
| E-N2 | 중간 | network.c에서 strlen()으로 CAN 바이너리 데이터 길이 체크 (NULL 없으면 오버플로) | network.c | ❌ |
| E-N3 | 낮음 | 캘리브레이션 for(i=1; i<50) — 첫 샘플 건너뜀 (의도/버그 불명확) | blinky_thread_entry.c | ❌ |

---

## 실행 우선순위

| 순서 | 이슈 | 대상 | 우선순위 | 상태 |
|------|------|------|----------|------|
| 1 | **Z1** 조향 클램핑 범위 수정 (-15→-7) | ZCU | P0 | ❌ |
| 2 | **Z2** StatusPublisher 종료 메시지 수정 | ZCU | P1 | ❌ |
| 3 | **E1** 워치독 타임아웃 단축 (6초→1초) | ECU | P0 | ❌ |
| 4 | **E5** Task 재생성 방지 | ECU | P1 | ❌ |
| 5 | **E-N2** strlen → dlc 사용 | ECU | P1 | ❌ |
| 6 | **E3** 캘리브레이션 실패 처리 | ECU | P1 | ❌ |
| 7 | **E2** 전역 변수 보호 | ECU | P1 | ❌ |
| 8 | **E-N1** ADC ScanStart 중복 제거 | ECU | P2 | ❌ |
| 9 | **E4** Zone 설정 외부화 | ECU | P2 | ❌ |
| 10 | **E-N3** 캘리브레이션 첫 샘플 확인 | ECU | P2 | ❌ |

---

## 빌드 환경 참고

- ECU 코드는 **Renesas e2 studio + FSP** 환경에서 빌드
- 이 프로젝트에서 직접 빌드 불가 (cross-compile 환경 필요)
- 코드 수정은 여기서 하고, e2 studio에서 빌드 + 플래시

---

## 성공 기준

1. 워치독 타임아웃 6초 → 1초 이하
2. ZCU 종료 시 ECU가 1초 내 자동 정지
3. Task 메모리 누수 제거
4. 조향 범위 ZCU-ECU 일관성 확보
