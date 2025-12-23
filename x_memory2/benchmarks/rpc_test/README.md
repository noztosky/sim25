# RPC Latency Benchmark (8kHz Target)

이 폴더는 AirSim RPC 방식의 통신 한계를 측정하기 위한 벤치마크 환경 세트입니다.

## 구성 파일
- `SIL_App_RPC.cpp`: 주파수 인자를 받는 벤치마크용 클라이언트 앱
- `PerfStats.hpp`: 제어 주기 대비 지연율(%)을 계산하는 통계 클래스
- `SimModeWorldBase.*`: `settings.json`에서 `PhysicsLoopPeriod`를 동적으로 읽도록 수정된 엔진 소스
- `build_rpc.bat`: 컴파일 스크립트

## 복구 및 실행 방법
1. **엔진 적용**: `SimModeWorldBase.h/cpp` 파일을 `Unreal/Plugins/AirSim/Source/SimMode/` 경로에 덮어쓰고 엔진을 리빌드합니다.
2. **설정**: `C:\Users\nodes\Documents\AirSim\settings.json`에 `"PhysicsLoopPeriod": 125000` (8kHz)를 추가합니다.
3. **빌드**: `build_rpc.bat`를 실행하여 `.exe`를 생성합니다.
4. **테스트**:
   - `.\build\SIL_App_RPC.exe 1000` (1kHz 기준)
   - `.\build\SIL_App_RPC.exe 4000` (4kHz 기준)
   - `.\build\SIL_App_RPC.exe 8000` (8kHz 기준)

## 실험 결론 (Baseline)
- **1k, 2k**: RPC 지연이 주기 내에 수렴하여 안정적임.
- **4k**: RPC의 물리적 지연(avg 0.2ms)이 한 주기(0.25ms)에 육박하여 지터와 샘플 유실 발생.
- **8k**: 통신 대역폭 포화로 인해 시스템 붕괴 및 실시간성 완전 상실.
