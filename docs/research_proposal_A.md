# 연구제안서 — 등가성 검증 기반 비행제어 펌웨어의 혼합정밀도 설계 방법

**(가제) "결정적 로그 재주입 등가성 bound를 이용한 비행제어 펌웨어의 블록별 정밀도 할당과 이기종 실기 검증"**

작성: 2026-07-12 (초안 v0.1)

---

## 1. 문제 정의

자원 제약이 큰 타깃(FPGA soft-core RISC-V 등)으로 비행제어 펌웨어를 이식할 때 float64→float32 정밀도 하향은 불가피하다. 그러나 **"어느 연산 블록을 어디까지 낮춰도 안전한가"를 결정하는 방법은 현재 존재하지 않으며, 실무는 임기응변에 의존한다.**

이 문제가 실재함을 보여주는 실증 사례가 있다: ArduPilot 개발진은 EKF3를 float32로 운용하다 **공분산 붕괴(variance collapse)로 인한 추정 불안정**을 겪었고, 로그 재주입(Replay)으로 이를 확인한 뒤 double 정밀도를 선택 가능하게 바꾸는 것으로 대응했다(ArduPilot PR #18008). 즉 세계에서 가장 널리 쓰이는 오픈소스 자동조종 장치조차 정밀도 결정을 **사후적·전역적·비정형적**으로 수행한다. 블록별로, 사전에, 정량 기준으로 결정하는 방법론이 빈자리다.

## 2. 관련 연구와 본 연구의 위치

| 연구군 | 대표 문헌 | 하는 것 | 못 하는 것 (본 연구와의 차이) |
|---|---|---|---|
| 정형 정밀도 튜닝 도구 | FPTuner (POPL'17), Daisy (2018), Darulova et al. (ICCPS'18), Isychev et al. (OOPSLA'25) | 수식 커널의 혼합정밀도 할당, 정적 오차 보증 | straight-line 커널·PC 한정. 2025년 현재도 FPBench 벤치마크 수준 — 실기 펌웨어 미적용 |
| 도구의 임베디드 제어 적용 | TAFFO→FOC 모터제어 (PARMA-DITAM'21) | 정밀도 튜닝을 실제 폐루프 제어에 적용 | 모터 FOC 한정, 비행제어/상태추정 아님, 오차 평가는 시뮬레이션 기반 |
| 제어 루프 정밀도 최적화 | Precision Switching Schedule (arXiv:2603.00616, 2026) | FP16/FP32 스위칭을 MIQP로 최적화 | **모델 기반** 결정(실측 아님), 피드백 제어기만(추정기 없음), 실기 검증 불명 |
| 저정밀 칼만 필터 | Compressed Gaussian Est. (Sensors'23), sigmaRho 필터 (NAVIGATION'19) | 저정밀 표현에서의 KF 정확도 분석/강건화 | 알고리즘 자체를 변경하거나 PC 평가에 한정 — 기존 펌웨어의 블록별 할당 아님 |
| SIL/PIL 등가성 검증 | MathWorks Back-to-Back 테스트, nanosat ADCS PIL (AEU'23), CubeSat PIL 프레임워크 (RiE'24) | 동일 입력의 sim-vs-target 허용오차 비교 + 온타깃 실행시간 측정 | **고정 정밀도의 합격/불합격 검증**에 그침 — 등가성 측정값을 설계 결정(정밀도 할당)에 사용하지 않음 |
| Sim-to-real 정량화 | UAV Testbed (Sensors 2026, 26(10):3188) | SITL vs 실비행 로그를 RMSE/JSD로 통계 비교 | **별개 비행 간 비교**(환경 gap 지배) — 동일 입력 재주입이 아니어서 코드/아키텍처 항 분리 불가 |
| RISC-V 비행제어 | TII PX4-on-RISC-V (2021), NEORV32 우주용 내결함 프로세서 (Aerospace) | RISC-V에서 자동조종 구동 가능성 입증 | 하드코어 SoC이거나 내결함성 목적 — 수치 정확도·지속가능 rate 특성화 전무 |

**빈 조합 (본 연구의 claim):** 결정적 로그 재주입으로 환경 항을 통제한 **등가성 bound를 정밀도 할당의 판정 기준으로 사용**하고, 이를 **이기종 실기(Cortex-M7 hard FPU + FPGA soft-core RISC-V)에서 수치·타이밍 양면으로 검증**한 연구는 확인되지 않음 (2026-07 기준, 7개 축 문헌 조사).

## 3. 제안 방법 (3단계)

**Stage 1 — 결정적 재주입 등가성 하네스.**
실비행/시뮬레이션에서 기록한 센서 로그를 동일 입력으로 (a) PC SIL, (b) 실기 타깃에서 재주입하고, 상태추정·제어 출력을 샘플 단위로 대조하여 ε-bound를 산출한다. 별개 비행을 통계 비교하는 기존 접근(Sensors 2026)과 달리, 동일 입력 재주입은 환경 항을 소거하고 **코드·컴파일러·FPU 아키텍처·정밀도가 만드는 차이만 분리**한다.

**Stage 2 — 블록별 정밀도 절단과 bound 맵.**
EKF를 연산 블록 단위(명목상태 전파 / 공분산 전파 / 칼만 이득 / 측정 갱신 등)로 분해하고, 블록별로 float64→float32 절단을 적용하며 Stage 1 하네스로 ε-bound를 측정한다. 산출물은 **"블록 × 정밀도 → 등가성 bound" 맵**과, 허용 bound가 주어졌을 때의 할당 판정 절차다. (ArduPilot PR #18008이 전역·ad hoc으로 수행한 것의 체계화.)

**Stage 3 — 이기종 실기 검증.**
판정된 할당을 두 아키텍처에서 검증한다: STM32H743(Cortex-M7, hard FPU, double 지원)과 NEORV32 soft-core RISC-V(FPGA, Zfinx float32). 수치 축은 Stage 1 하네스로, 시간 축은 DWT/사이클 카운터 기반 **타깃별 최대 지속 가능 추정 rate 곡선**으로 측정한다. (확장) 판정 할당으로 구성한 soft-core FC의 실비행으로 최종 검증 루프를 닫는다.

## 4. 기존 실험 계획과의 관계

기 수립된 실험 4종이 방법의 검증 구조로 재배치된다 — 폐기되는 작업 없음:

- 실험 ① (로그 재주입 등가성) → Stage 1의 하네스 그 자체
- 실험 ③ (double→float32 ε 선측정) → Stage 2의 블록별 확장
- 실험 ② (온타깃 타이밍/rate) → Stage 3의 시간 축 검증
- 실험 ④ (실비행) → Stage 3의 확장 검증

## 5. 기대 기여

1. 비행제어 펌웨어의 정밀도 할당을 **측정된 등가성 bound로 판정하는 최초의 체계적 방법**과 그 공개 구현 (기존: 정적 커널 분석 또는 ad hoc 전역 전환)
2. 동일 코드의 **이기종 아키텍처 간 수치 등가성 bound의 최초 정량 공표** (hard FPU vs soft-core, double vs float32, 블록별)
3. 자동조종 펌웨어의 **타깃별 최대 지속 가능 추정 rate 특성화 곡선** (기존 문헌: 단일점 시연 또는 포럼 수준 논의만 존재)

## 6. 일정 (안)

| 기간 | 작업 |
|---|---|
| ~3주 | H743 포팅 + Stage 1 하네스 (베이스라인 확보) |
| ~6주 | 탈Eigen·블록 분해 + Stage 2 bound 맵 |
| ~10주 | Tang 25K NEORV32 브링업 + Stage 3 교차 검증 |
| 이후 | (확장) 커스텀 보드·실비행, 저널 투고 |

## 7. 사전 확인 필요 사항 (정직성 항목)

- Sensors 2026 26(10):3188 원문 정독 — "onboard resource behavior" 절이 루프 타이밍까지 다루는지 확인 (related work 1순위)
- arXiv:2603.00616 원문 정독 — 실기 검증 여부 최종 확인
- ArduPilot PR #18008 및 후속 논의 전수 확인 — 동기 사례로 인용

## 참고문헌 (핵심)

1. ArduPilot PR #18008 — EKF3 single/double precision, Replay 기반 검증
2. Darulova, Horn, Sharma. *Sound Mixed-Precision Optimization with Rewriting.* ICCPS 2018
3. Chiang et al. *Rigorous Floating-Point Mixed-Precision Tuning* (FPTuner). POPL 2017
4. Isychev et al. *Cost of Soundness in Mixed-Precision Tuning.* OOPSLA 2025
5. *The Impact of Precision Tuning on Embedded Systems Performance: FOC Case Study.* PARMA-DITAM 2021
6. *Precision Switching Schedule for Efficient Control Implementations.* arXiv:2603.00616, 2026
7. *Compressed Gaussian Estimation under Low Precision Numerical Representation.* Sensors 23(14), 2023
8. Grewal, Andrews. *sigmaRho filter — Practical KF for mission-critical applications.* NAVIGATION 66(2), 2019
9. *A UAV Testbed for Diagnosing Hardware Vulnerabilities: Quantifying Sim-to-Real Discrepancies in PX4 Flight Logs.* Sensors 26(10):3188, 2026
10. MathWorks. *SIL and PIL Simulations / Back-to-Back Equivalence Testing* (산업 관행 근거)
11. *Design and analysis of a nanosatellite ADCS using processor-in-the-loop.* AEU-IJEC, 2023
12. TII SSRC. *Secure PX4 Stack on RISC-V Drone.* 2021
