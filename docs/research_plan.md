# 석사논문 연구계획 — 결정적 SIL과 이기종 실기 타깃 간 전이 등가성 정량 검증

> 최종 갱신: 2026-07-04
> 발표덱: [thesis_plan_SIL_to_STM32H7.pptx](thesis_plan_SIL_to_STM32H7.pptx)

---

## 1. 논문 한 줄 요약

**"단일 소스 비행제어 코어(24-state EKF + PID)가 결정적 SIL에서 검증한 결과 그대로 실기(STM32H7 / soft RISC-V)에서 동작함을 오차 bound(ε)와 실측 타이밍으로 정량 증명하고, soft RISC-V FC로 실비행까지 수행한다."**

- 주인공: **soft RISC-V FC (Tang Primer 25K, 실비행까지)**
- 베이스라인 + 일정 보험: **STM32H743** (먼저 수행, 2~4주, 홈그라운드)
- 관통 도구: **SIL Log Replay 등가성 파이프라인**

## 2. 연구 질문 (RQ)

> **RQ. SIL에서 검증한 비행제어 코드는 실기에서 어떤 오차 범위(bound)로 동일하게 동작하는가?**

- **RQ1 — 수치 등가성**: 동일 입력에 대한 EKF·PID 출력 차이의 ε-bound
- **RQ2 — 실시간 실현가능성**: 각 타깃에서 제어주기 마감(deadline) 준수 여부, 타깃별 최대 지속 가능 EKF rate

## 3. 확립된 사실 (재조사 불필요)

### 3.1 SIL 무결성 검증 완료 (자체 데이터)

| 항목 | 값 |
|---|---|
| Δt 간격 종류 | **1종** (전 rate 단일값) — 1kHz: 999,936ns × 18,762 / 4kHz: 250,112ns / 8kHz: 124,928ns |
| 데이터 결손 | **0** (1k/4k/8k 전 구간) |
| 타임스탬프 지터 | 평균 0.064~0.112 μs |
| RTF (wall-clock) | 1kHz ≈ 0.73~0.77 / 8kHz ≈ 0.31 — **실시간 아님. "실시간 달성" 주장 금지** |
| 두 시간축 분리 | FC가 입력받는 유일한 시간 = Simulation Time(lockstep). RTF 저하는 계산 결과에 무영향 |

### 3.2 문헌조사 결과 — novelty 아님 (인용하고 그 위에 설 것)

| 이미 있는 것 | 출처 |
|---|---|
| HAL 단일 코드 (sim+펌웨어), lockstep 결정성 | ArduPilot/PX4 SITL (성숙, 표준) |
| 실보드에 펌웨어 + 시뮬 센서 구동 | ArduPilot Sim-on-Hardware |
| 로그 재주입으로 EKF 재현 | ArduPilot Replay (디버깅 도구) |
| SHM 저지연 IPC | RTX/reflective memory HIL (수십 년 확립) |
| "고rate가 좋다" | PX4 공식 문서 — 단 **프론트엔드(자이로 필터) 한정**, 4kHz+는 H7 전제 |
| RL 제어기 + FPGA 실비행 | Azem et al. arXiv:2403.18703 (2024, Artix-7) |
| Zynq 기반 EKF/제어 FPGA 구현 | 다수 — 단 대부분 "구현 보고서"(자원표+속도) |

### 3.3 남은 방어선 (본 연구의 자리)

- **sim↔실기 등가성 bound의 정량 공표** — 선례가 얇음
- **온타깃 타이밍 검증** — "SITL은 타이밍·I/O 지연을 못 잡는다"는 문헌상 인정된 gap
- **soft RISC-V FC 실비행** — 드문 실증
- 플랫폼 선택(Tang vs Zynq)은 **novelty 0** — 순수 공학 결정

### 3.4 핵심 설계 원리

- **EKF 2단 분리**: rate PID는 자이로 직결 1kHz 유지 / 명목상태 전파 1kHz / **공분산+융합 100Hz** (ArduPilot `EKF_TARGET_DT` 관행과 일치. 8kHz EKF는 불필요 — 고rate 실익은 프론트엔드에만)
- 이 결정으로 soft RV32에서 EKF가 가능권 진입 (100Hz float32 ≈ 5–10 MFLOPS 필요)

## 4. 확정된 결정 사항

| 결정 | 확정값 | 근거 |
|---|---|---|
| 플랫폼 | **Tang Primer 25K** (GW5A-LV25MG121) | 제품 계보: 단가 ~$10대, 23×18mm BTB 모듈, FPGA 양산 노선(HDZero 선례), ASIC(IDEC MPW) 정합. Zynq-7020은 검토 후 배제(관련연구 레퍼런스 용도) |
| soft core | **NEORV32 + Zfinx(float32)** | 문서화 최고, FC 요구와 주변장치 정합(SPI 멀티CS·PWM·NEOLED·DMA), 순수 VHDL(AI 친화), OpenOCD/gdb. 성능 부족 시 VexRiscv 에스컬레이션 |
| 예제 코어 | 브링업 검증(툴체인·JTAG)에만 사용 | PicoRV32 계열은 FPU 없음(막다른 길), Cortex-M1 EMPU는 ARM(ASIC 경로 차단) — 배제 |
| H743 역할 | 베이스라인 + 일정 보험, **먼저 수행** | 대표성 아닌 리스크 산수: Tang 경로는 직렬 미지수, H7은 3주 홈그라운드. 모든 표의 "산업표준 대비" 비교 열 |
| 정밀도 | CPU는 float32(Zfinx), double은 soft-float 불가(~1 MFLOPS) | double→float32 ε는 PC에서 선측정 (그 자체가 실험 데이터) |
| FP64 가속기 | **박사 카드** — 지금 설계 안 함 | 버스(XBUS/CFS) 주변장치로 구현 (프로세서 레벨 아님). DSP 18×18 × 9~12개/FMA, 파이프라인 1기 ≈ 100+ MFLOPS |
| NPU | 25K에 안 들어감 | Tang Mega 138K(GW5AST — 하드 RISC-V AE350)급 이후. "Zynq 등가물"은 Zynq가 아니라 Mega 138K |
| AI 활용 원칙 | **"AI가 만들고, 데이터가 검증하고, 본인이 설명한다"** | 코드는 replay 파이프라인이 검증. 심사 방어는 블록 수준 설명 가능까지. 논문 텍스트 AI 사용은 학교 규정 확인 |

## 5. 하드웨어 스펙 (확인 완료)

### Tang Primer 25K — GW5A-LV25MG121 하드 블록

| 블록 | 수량 | 용도 |
|---|---|---|
| LUT4 / FF | 23,040 / 23,040 | NEORV32 예산 ~6.5–8.5K → 65% 여유 |
| BSRAM | 1,008Kb(126KB), 56블록 | IMEM/DMEM — **탈Eigen 필수 (Eigen 코드 팽창으로 초과 위험)** |
| 분산 SSRAM | 180Kb | |
| 18×18 곱셈기 | 28개 | FP64 FMA 1~2기 가능 (박사 단계) |
| PLL | 6개 | |
| I/O | 8뱅크, GPIO 75핀 | |
| MIPI D-PHY 하드코어 | 4-lane (RX 1.6G/TX 2.0Gbps) | **SoC 비전(카메라·영상링크)에 직결** |
| NOR Flash | 64Mb | 비트스트림 + XIP |
| 하드 CPU | 없음 | → NEORV32 |

- 모듈 23×18mm, **2×60P BTB 커넥터** → 기존 H743 코어보드+인터페이스 보드 패턴 그대로 커스텀 캐리어 제작 가능
- Dock(64×40mm, USB-C JTAG+UART): 개발 + 1차 비행용 (F450급 탑재 여유)

### 주변장치 매핑

| 요구 | 해법 |
|---|---|
| ICM-42688-P + BMI088급 IMU 2개 | 내장 SPI 1개 + CS 2줄 (공유 버스) |
| PWM 8ch (모터4+서보3) | 내장 PWM (채널 수 generic) |
| NeoPixel 1ch | **내장 NEOLED** (WS2812 전용 하드웨어) |
| MAVLink / GPS UBX | 내장 UART0 / UART1 |
| **SBUS** | 내장 UART 부족(2개 고정) → **XBUS에 커스텀 반전 RX 모듈** (~200 LUT, generic 증설 불가) |
| 불필요 블록 | generic으로 제거 (`IO_TRNG_EN => false` 등) — 합성 자체가 안 되어 LUT 0 |

## 6. 실험 계획

| # | 실험 | 내용 | 지위 |
|---|---|---|---|
| ① | **Replay 등가성** | 동일 센서 로그 → SIL(x86) vs H743(ARM) vs RV32(soft) 출력 샘플별 비교 → RMSE·최대오차·발산 여부 = ε-bound | **필수 — 킬러 그래프** |
| ② | **온타깃 타이밍** | DWT/사이클카운터로 EKF·PID 스텝 실행시간 실측 → "타깃별 최대 지속 가능 EKF rate" 곡선 | **필수** |
| ③ | 내부 Baseline | 큐 버퍼링 없는 구현의 결손율 → 설계 정당화 | 정당화 (1일) |
| ④ | **실비행 대조** | soft RISC-V FC 실비행 로그 vs SIL 예측 궤적 중첩 | 주인공 실증 |

- ①+②만으로 학위논문 성립. ①②③ = 국내 학술대회 확실. ④까지 = 국내 학회지(KCI) 안정권
- 주의: "비트 동일"이 아니라 **"ε-bounded 등가"**를 주장 (x86↔ARM↔RV32 부동소수 차이 1e-6~1e-4는 정상 — 그 bound 측정 자체가 결과)
- ④에서 괴리가 커도 실패가 아님 — **"괴리 정량화 + 원인 분해"로 프레이밍** (등가성 논문에서는 괴리 측정치 자체가 결과)

## 7. 실행 로드맵

```
0.  [공통 뿌리 — 다음 작업] EST_EKF24 리팩토링
    탈Eigen(고정 24×24 C 배열) + 명목/공분산 2-rate 분리 + float/double 템플릿화
    → 현 Eigen판을 golden reference로 replay 검증
    ※ SIL_App.cpp는 현재 매 샘플 전체 update_imu 호출 → 분리 리팩토링 필요

1.  [보험, 2~4주] H743 포팅 (기존 코어보드+캐리어 재사용, 코어만 단일 소스 교체)
    → 실험 ①② 데이터 확보 = 논문 최소 성립

2.  [병행 가능] float32 vs double ε-bound PC 선측정 (하드웨어 불요)

3.  Tang 25K 브링업: 예제 비트스트림(툴체인 검증) → NEORV32 UART·타이머·1kHz 루프

4.  동일 코어 RV32 포팅 → 실험 ①② 3번째 열 (크로스 ISA 등가성)

5.  커스텀 SBUS RX + 주변장치 통합 → dock으로 1차 실비행 (프롭제거→테더→저고도)

6.  [이후] 비행용 GW5A-25 커스텀 캐리어 보드 (BTB, 본인 주특기)

7.  [박사/SoC 로드맵] PL FP64 MAC 가속기 → Tang Mega 138K(NPU·영상) → IDEC MPW 자체 SoC
    최종 비전: FC + 디지털 영상링크(HDZero 방식) + NPU 통합 SoC
```

## 8. 논문 구성 (5장)

1. **서론** — 문제 정의 · RQ
2. **관련연구** — SITL·HIL·Sim-to-Real (§3.2 정직한 인용 — **실험 전에 먼저 쓸 것**)
3. **시스템** — SIL 플랫폼 · 3계층 구조(application–cuduino–HAL) · 단일 소스 코어 · NEORV32 SoC
4. **실험** — 등가성 · 타이밍 · Baseline · 실비행
5. **결론** — 기여 · 한계 · 향후 연구(FP64 가속기·NPU·SoC)

### 기여 (Contributions)

- **C1** 단일 소스 SIL→이기종 실기 검증 파이프라인의 완결 구축
- **C2** 전이 등가성 오차 bound의 정량 제시 (공표 선례가 얇은 영역)
- **C3** 온타깃 실시간 실현가능성(타깃별 최대 EKF rate) 실측 + soft RISC-V FC 실비행 실증

### 포지셔닝 문장

> "ArduPilot을 능가"가 아니라 — **전 계층(물리엔진→SHM→코어→보드→비행)을 통제·계측할 수 있는 연구 플랫폼의 완결된 정량 검증.** 기존 성과는 숨기지 않고 인용하며, 그 위에 선다.

## 9. 심사 방어 준비 (예상 질문)

| 예상 질문 | 준비된 답 |
|---|---|
| "ArduPilot 쓰면 되는데 왜 직접 만들었나?" | 연구를 위해 전 계층이 손에 있어야 함 (타이밍·데이터경로 계측·개조의 완전한 통제) |
| "SITL과 뭐가 다른가?" | §3.2 표 — 등가성 bound 정량 공표·온타깃 타이밍 검증이 없음 |
| "EKF 공분산 식을 설명해보라" | **EKF24는 오픈소스/AI 기반임을 논문에 명기** + Solà(arXiv 1711.02508)로 블록 수준 설명 가능까지 학습 (재구현 아닌 설명이 목표) |
| "실비행 검증은?" | 실험 ④. 없이 갈 경우 스코프를 "연산 등가성+실시간성"으로 명시적 한정 |
| "baseline이 공정한가?" | 동일 물리엔진·동일 rate·transport만 차이 명시 |

## 10. 금지 목록 (과장 주장 — 심사에서 반드시 깨짐)

- ❌ "실시간 8kHz 달성" (자체 RTF 0.31로 반증됨)
- ❌ "기존 SITL은 구조적으로 불가능" (lockstep이면 가능, 느릴 뿐)
- ❌ "Zero-Code-Change는 우리만의 무기" (ArduPilot HAL이 이미 함)
- ❌ "SHM으로 속도 개선"을 헤드라인으로 (선행연구 다수 — 구현 수단으로 한 문단만)
- ❌ "Tang/Gowin 최초 논문" (보드 선택은 novelty 0)
- ❌ Gazebo와 성능 경쟁 (물리엔진 무게가 달라 인과 증명 불가 — 비교는 transport 격리 실험으로만)
