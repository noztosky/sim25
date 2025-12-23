# AirSim Engine Modification for Dynamic Physics Period

본 문서는 시뮬레이터 물리 엔진의 업데이트 속도를 동적으로 조정하기 위해 AirSim 커널에 적용된 수정 사항을 기록합니다.

## 1. 수정 배경
기존 AirSim은 물리 루프 주기(`physics_loop_period_`)가 코드 내에 하드코딩되어 있어, 주파수 변경 시마다 언리얼 프로젝트의 리빌드가 필요했습니다. 벤치마크 효율성을 높이기 위해 이를 `settings.json`에서 직접 제어할 수 있도록 개선하였습니다.

## 2. 주요 수정 사항

### 2.1 SimModeWorldBase.h
- `physics_loop_period_` 멤버 변수의 기본값을 1ms(1,000,000ns)로 표준화.
- 외부(SimModeWorldBase.cpp)에서 이 값을 안전하게 덮어쓸 수 있도록 구조 확인.

### 2.2 SimModeWorldBase.cpp (핵심 로직)
`initializeForPlay()` 함수 내에 다음 로직을 추가하여 시뮬레이션 시작 시 설정을 주입하도록 수정되었습니다.

```cpp
// settings.json에서 "PhysicsLoopPeriod" 항목을 읽어옴 (기본값: 현재 엔진 설정값)
long long period = msr::airlib::Settings::singleton().getInt("PhysicsLoopPeriod", (int)getPhysicsLoopPeriod());

// 읽어온 값을 물리 커널 주기에 즉시 반영
setPhysicsLoopPeriod(period);
```

## 3. 주파수별 설정 가이드 (settings.json)
수정 후에는 아래와 같이 `PhysicsLoopPeriod`(단위: Nanoseconds) 값을 조정하여 엔진 속도를 제어할 수 있습니다.

- **1 kHz**: `"PhysicsLoopPeriod": 1000000`
- **4 kHz**: `"PhysicsLoopPeriod": 250000`
- **8 kHz**: `"PhysicsLoopPeriod": 125000`

## 4. 시사점
이 수정을 통해 벤치마크 실험 시 **시뮬레이터 재빌드 없이 `settings.json` 수정 및 시뮬레이터 재시작만으로** 다양한 환경에서의 통신 성능을 즉각적으로 비교 분석할 수 있게 되었습니다. 이는 논문 실험의 변수 통제 및 재현성을 확보하는 핵심 설계 요소입니다.
