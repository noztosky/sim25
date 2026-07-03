# 논문 로드맵 — 크로스플랫폼 FC 프레임워크 (Windows-SIL / STM32 / RISC-V)

작성: 2026-07-03

## 논문 핵심 주장 (한 문장)

> **단일 FC 코드베이스(HAL 추상화)가 Windows-SIL / STM32 / RISC-V에서
> 동일한 타이밍 시맨틱(1–8kHz 등간격 dt)으로 동작함을 실측으로 검증한
> 크로스플랫폼 FC 개발 프레임워크**

## 현재까지 확보된 것 (2026-07-03 기준)

- AirSim(UE4.27 Blocks) 수정: FastPhysicsEngine SHM publish, 시뮬시간 정렬
- SHM(lock-free ring, x_xsim) 기반 텔레메트리 경로: RPC 대비 정량 비교 자료 일부
- 지표 체계의 발견: **sim-time 등간격성**(무결성) vs **wall-time RTF/지연**(실시간성) 분리
  - sim-time 축: 1k/4k/8k 모두 dt 상수(999.94/250.11/124.93µs), 지터 ~0.1µs, 결손 0
  - wall-time 축: RTF < 1, 요동 큼 → SIL(lockstep) 모델에서는 무관, HIL에서만 요건
- 계측 인프라: physics_jitter 로그(엔진), SIL perf 로그(소비자), 순수 타이머 벤치(x_memory2)
- 정지 상태 값 검증: 타임스탬프 단조/결손 0, baro=GPS=EKF 수렴, 물리값 정합

## 알려진 결함 (수정 필요)

1. **[P0] 소비 루프 샘플 스킵**: `IMU_SimSHM::read()`가 밀린 큐를 비우고 최신 1개만
   반환 → SIL 처리 루프가 느리면 정확히 절반 스킵 (1kHz 설정 시 실수집 500Hz,
   flight_log 간격 2ms로 실측 확인됨)
2. **[P1] 설정 파일 불일치**: 루트 settings.json=ScalableClock,
   Blocks 프로젝트=SteppableClock — launch 경로에 따라 다른 클럭 로드
3. **[P1] 동적 검증 부재**: 지금까지 전부 정지 상태 데이터 (acc/gyro 변화 0)

---

## Phase 1 — SIL 완결 (~2주) → 국내 학회 포스터 가능선

| # | 작업 | 완료 기준 |
|---|---|---|
| 1-1 | 동적 기동 검증 (takeoff→호버→자세 스텝) | EKF vs GT 오차(RMSE), 고도 5m 추종, gyro/acc 변화 확인 |
| 1-2 | 1k/4k/8k × 동일 시나리오 반복 실험 | 각 5회 이상, 평균±표준편차 표 |
| 1-3 | 소비 루프 스킵 수정 (큐 전량 순차 처리) | flight_log 간격 = 1/target_hz 100%, 스킵 0 |
| 1-4 | 지표 정의 문서화 (sim-time vs wall-time) | 방법론 섹션 초안 |

## Phase 2 — Baseline 비교 (+2~4주) → 국내 구두 / 국제 워크숍

| # | 작업 | 산출물 |
|---|---|---|
| 2-1 | PX4 SITL(lockstep) 동일 시나리오 비교 | 레이트 상한/dt 지터/지연 비교표 |
| 2-2 | RPC vs SHM vs 전용스레드 정량 비교 완성 | 아키텍처 정당화 표 (기존 벤치 재활용) |
| 2-3 | (선택) RTF>1 가속 시뮬 데모 | lockstep 장점 실증 |

## Phase 3 — 실기 이식 (+1~2개월) → 국제 학회 도전선

| # | 작업 | 비고 |
|---|---|---|
| 3-1 | STM32에서 동일 EKF/PID 구동 (HAL만 교체) | **핵심 증거** — 코어 로직 diff 0 주장 |
| 3-2 | STM32 실측 1k~4k dt 지터/CPU → SIL과 병렬 표 | 타이밍 등가성 실증 |
| 3-3 | 동일 시나리오 SIL vs 실기(HIL) 응답 비교 | sim-to-real 근거 |
| 3-4 | RISC-V는 여력 시 (2플랫폼으로도 성립) | 무리하지 않음 |

## 판정선

```
최소선(국내):   Phase 1 + 2-2                 ← ~1개월
권장선(국제):   Phase 1 + 2 + 3-1, 3-2        ← STM32 실기 데이터가 분수령
욕심선(RA-L):   전부 + RISC-V + HIL 비행
```

**최대 리스크**: STM32 보드/고ODR IMU(예: ICM-42688, 8kHz) 확보 일정 → 최우선 확정 필요.
