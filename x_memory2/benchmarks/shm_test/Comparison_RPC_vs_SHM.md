# RPC vs. SHM Performance Comparison Analysis

## 1. Overview
This report compares the performance of **RPC (Remote Procedure Call)** and **SHM (Shared Memory)** for high-frequency IMU data acquisition (1kHz to 8kHz) in AirSim. The goal is to highlight why SHM is essential for high-fidelity flight control simulation.

## 2. Jitter Comparison Table (Target Frequency vs. Stable Ratio)
"Stable" is defined as the percentage of samples falling within the **< 5%** delay bucket.

| Frequency | RPC Stable Ratio (<5%) | SHM Stable Ratio (<5%) | Improvement |
| :--- | :--- | :--- | :--- |
| **1 kHz** | 99.93% | **100.00%** | +0.07% |
| **4 kHz** | 91.99% | **99.99%** | **+8.00%** |
| **8 kHz** | 9.10% | **99.98%** | **+90.88%** |

## 3. Histogram Data Comparison (Counts)

### 3.1 1kHz Comparison
| Bucket | RPC (Counts) | SHM (Counts) |
| :--- | :--- | :--- |
| **[0] < 5%** | 9,527 | **9,952** |
| **[1-4] Lag** | 0 | 0 |
| **[5] >= 30%** | 7 | 0 |

### 3.2 4kHz Comparison
| Bucket | RPC (Counts) | SHM (Counts) |
| :--- | :--- | :--- |
| **[0] < 5%** | 30,789 | **39,818** |
| **[1] < 10%** | 720 | 1 |
| **[2] < 15%** | 499 | 0 |
| **[3] < 20%** | 407 | 0 |
| **[4] < 30%** | 485 | 0 |
| **[5] >= 30%** | 571 | 1 |

### 3.3 8kHz Comparison
| Bucket | RPC (Counts) | SHM (Counts) |
| :--- | :--- | :--- |
| **[0] < 5%** | 5,014 | **79,645** |
| **[1] < 10%** | 3,679 | 1 |
| **[2] < 15%** | 4,191 | 1 |
| **[3] < 20%** | 4,273 | 0 |
| **[4] < 30%** | 7,491 | 3 |
| **[5] >= 30%** | 30,426 | 6 |

## 4. Key Findings
1. **The "Cliff" Effect in RPC**: At 8kHz, RPC performance completely collapses, with **over 90%** of samples suffering from significant jitter and lag. This makes the simulation unusable for high-frequency control.
2. **SHM Determinism**: SHM maintains near-perfect timing (**99.98% stable**) even at the extreme 8kHz edge case. The jitter is virtually non-existent regardless of the frequency.
3. **Throughput**: SHM successfully delivered nearly **100%** of the target samples (79.6k/80k), whereas RPC could only manage around **69%** (55k/80k) due to network stack bottlenecks.

---

# RPC vs. SHM 성능 비교 분석 보고서

## 1. 비교 요약
8kHz 고주파 시뮬레이션 환경에서 RPC와 SHM의 성능 격차는 극명합니다. 특히 주파수가 높아질수록 RPC는 통신 오버헤드를 견디지 못하고 성능이 급격히 붕괴하는 반면, SHM은 일정한 결정성(Determinism)을 유지합니다.

## 2. 주요 지표 비교
- **1kHz**: 두 방식 모두 안정적이나, SHM이 지터가 전혀 없는 완벽한 수치를 보여줌.
- **4kHz**: RPC는 약 8%의 샘플에서 지연이 발생하기 시작함. SHM은 여전히 99.99% 안정.
- **8kHz**: RPC는 안정적인 샘플이 **9%**에 불과하며 사실상 시뮬레이션 불가능 상태. **SHM은 99.98%의 샘플이 5% 이내의 지연**을 유지하여 완벽한 실시간 성능을 정명함.

## 3. 결론
고속 드론 제어나 물리 엔진과 동기화된 정밀 시뮬레이션을 위해서는 **SHM(공유 메모리) 아키텍처 도입이 유일한 정답**임을 데이터가 증명하고 있습니다.
