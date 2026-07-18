# EFT Z30 농업용 드론 프레임 + 튜닝 가능한 비행제어

> 2026-07-18 구현. SIL_App(SHM 기반 비행제어) + AirSim 포크에 40kg 농업용 드론(EFT Z30,
> 빈 탱크, 앞/뒤 무게 불균형)을 config로 추가하고, 재빌드 없이 PID를 튜닝할 수 있는
> 파라미터 파일·런타임 명령을 붙였다. (석사논문 SIL→H7 실험과는 별개 트랙)

---

## 1. 무엇이 추가됐나

### (A) EFT Z30 프레임 config — AirSim 코어 물리는 안 건드림
`AirLib/include/vehicles/multirotor/MultiRotorParams.hpp` → `setupFrameEFTZ30()`

| 항목 | 값 | 근거 |
|---|---|---|
| 질량 | 40 kg | 자중 29.8 + 배터리 10 (제조사 스펙) |
| 휠베이스 | 2025 mm (팔 1.0125 m) | 대각선 모터간격, 검증됨 |
| 프로펠러 | 41135 (D=1.041 m) | 축당 18 kgf → max_rpm 2005 |
| 호버 스로틀 | ~0.555 | mass·g / (4·max_thrust) |
| 관성 | Ixx 4.45 / Iyy 5.05 / Izz 8.02 kg·m² | 팔+모터가 지배 |
| CG 오프셋 | 5 cm 뒤쏠림 (`cg_offset_x = -0.05`) | 빈 탱크 앞/뒤 불균형. **실물 밸런스 재면 이 값만 갱신** |
| 모터 지연 | 30 ms (`control_signal_filter_tc = 0.03`) | 120 ms는 자세루프 위상지연으로 발산 → 30 ms로 해결 |

- 프레임 선택: `AirSimSettings.hpp`에 `Model` 필드 파싱 추가 → settings.json에서
  `"Model": "EFTZ30"` 로 선택. 이 줄 지우면 기존 1kg 쿼드(논문 기준선)로 복귀.
- **주의**: `AirLib/include`가 원본. 편집 후 플러그인 사본 2곳 수동 동기화 필요
  (`Unreal/Plugins/...`, `Unreal/Environments/Blocks/Plugins/...`) → buildue.bat이 복사.

### (B) 프레임 프로파일 — SIL_App
`x_memory2/apps/SIL_App.cpp`

- 실행 인자 `z30` 으로 Z30용 게인 프로파일 선택. 없으면 기존 1kg 게인(기준선 보존).
- hover_throttle·자세게인·rate PID·고도 PID가 전부 프로파일 분기.

### (C) 파라미터 파일 로딩 — 재빌드 없이 튜닝
- 시작 시 `pid_params_z30.txt`(z30) 또는 `pid_params.txt`(기본)를 읽어 게인을 덮어씀.
- 파일 없으면 내장 기본값 폴백. 일부 키만 있어도 나머지는 기본값 유지.
- 파일 위치: 작업 디렉터리(`x_memory2\build\`). `params=<경로>` 인자로 변경 가능.
- 시작 시 로드된 값 전부 콘솔 출력.
- **build/ 는 gitignore 대상** → 파일 자체는 버전관리 안 됨(작업 파일). 필요시 밖으로 이동.

### (D) 런타임 명령 2종 — 재시작 없이
UDP:5005 로 전송 (SendCmd.exe 또는 sendcmd_gui.py 사용).

| 명령 | 동작 |
|---|---|
| `pid` | 파라미터 파일 다시 읽어 게인·hover 즉시 재적용 (콘솔에 새 값 출력) |
| `reset` | 이륙 전 상태로: 기체 스폰위치 복귀(RPC reset) + 자동조종 OFF + PID 적분기·목표값·yaw기준 클리어 + EKF 재초기화 |

---

## 2. 실행 & 튜닝 워크플로우

### 빌드
```cmd
:: SIL_App / SendCmd 만 (게인·명령 코드 변경 시, ~10초)
cd d:\xlab\sim25\x_memory2
build.bat

:: 프레임 물리(MultiRotorParams 등) 변경 시 — AirLib + UE 리빌드 필요
cd d:\xlab\sim25
msbuild AirLib\AirLib.vcxproj /p:Configuration=Release /p:Platform=x64
buildue.bat
```

### 실행 순서 (규칙)
```cmd
:: 1) 시뮬 (기체 뜰 때까지 대기)
sblocks.bat

:: 2) 컨트롤러 — 숫자 N = 물리 레이트 (settings.json PhysicsLoopPeriod)
::    250000 → 4000,  1000000 → 1000.  안 맞으면 SIL_App 멈춤!
cd x_memory2\build
SIL_App.exe 4000 z30

:: 3) 이륙
cd x_memory2\build
SendCmd.exe a 5 10
```
- 합치기: `SIL_App.exe 4000 z30 takeoff 5` (t=0부터 5m 유지)
- SendCmd는 발신 전용 — 수신자(SIL_App) 없으면 무의미.

### 튜닝 루프 (전부 재시작 없음)
```
1. SIL_App.exe 4000 z30      ← 한 번만 실행
2. SendCmd a 5 10            ← 이륙 (거동 관찰)
3. SendCmd reset             ← 원위치 + 초기화
4. pid_params_z30.txt 수정 → 저장
5. SendCmd pid               ← 새 게인 즉시 반영
6. SendCmd a 5 10            ← 다시 이륙 → 2로 반복
```

---

## 3. 파라미터 파일 레퍼런스 (`pid_params_z30.txt`)

```
hover_throttle   # 40kg 호버 기준 스로틀 (~0.74). 높으면 이륙 공격적
alt_p/i/d        # 고도 PID
alt_ilim         # 적분 클램프
alt_min/max      # 스로틀 출력 하/상한. max 작게 유지해야 collective가 1.0에 안 붙어 자세 여유 확보
alt_dfilt        # D 저역통과 (Hz)
rate_p/i/d       # roll/pitch 내부 rate PID. rate_d = 진동 감쇠
rate_clamp       # 축당 모터 권한(±). 너무 넓으면 과여기→뒤집힘
rate_ilim, rate_dfilt
again_roll/pitch/yaw   # 외부 각도→rate 게인
yaw_p/i/d, yaw_clamp
```

---

## 4. 명령 레퍼런스 (SendCmd → UDP:5005)

| 명령 | 예 | 의미 |
|---|---|---|
| `a <m> <s>` | `SendCmd a 5 10` | 고도 유지(이륙). 목표고도 영구, 자동조종 ON |
| `r <deg> <s>` | `SendCmd r 30 1` | 롤 기동 (지속 s초) |
| `p <deg> <s>` | `SendCmd p 20 1` | 피치 기동 |
| `y <deg> <s>` | `SendCmd y 45 2` | 요 기동 |
| `pid` | `SendCmd pid` | 파라미터 파일 재로드 |
| `reset` | `SendCmd reset` | 이륙 전 초기화 |

주의: r/p/y는 첫 `a`(이륙) 이후에만 유효. `reset`은 시뮬 RPC(41451)가 열려 있어야 기체 복귀.

### GUI 제어 도구 (`sendcmd_gui.py`) 사용법
기존 CLI 제어 방식(`SendCmd.exe`) 외에, 그래픽 기반의 편리한 튜닝 및 기동 테스트를 위해 파이썬 Tkinter 기반의 GUI 도구를 제공합니다.
* **실행 방법**:
  ```cmd
  cd d:\xlab\sim25\x_memory2
  python apps/sendcmd_gui.py
  ```
* **제공 기능**:
  - **Connection Settings**: 대상 IP(기본 `127.0.0.1`)와 포트(기본 `5005`) 설정.
  - **System Commands**: `Reload PID` 및 `Reset Drone` 원클릭 버튼 지원.
  - **Command Builder**: 축(a, r, p, y)과 제어값(Value), 지속 시간(Duration)을 입력하고 `Send Manual Command`를 눌러 송신.
  - **Preset Commands**: 이륙(5m/10m) 및 기동(Roll, Pitch, Yaw)에 대한 자주 사용하는 프리셋 버튼 지원.
  - **Transaction Log**: 송신한 명령과 전송 성공/실패 여부를 콘솔 로그로 실시간 모니터링 및 복사 가능.

---

## 5. 현재 상태 & 남은 일

**해결됨**: 이륙 시 NaN 발산 (원인 = 모터지연 120ms 위상지연 → 30ms).

**미완**: Z30 비행 게인 튜닝. 이륙 순간 자세(pitch/roll) 오버슈트로 **뒤집힘**.
- 발견한 실패 모드: 고도 PID가 collective를 1.0에 railing → 자세 제어용 모터 여유 상실 →
  뒤 모터 포화 → 넘어짐. (`alt_max` 작게 + rate 감쇠가 방향)
- 눈먼 로그 튜닝의 한계로 진행 중. 파라미터 파일 + pid/reset 명령으로 반복 튜닝하는 인프라는 완료.

**헛다리(기록)**: 뒤무게 불균형(5cm라 완만한 트림만)·지면충돌·body_box 과대
(드래그는 프로펠러 면적 13.6m²가 지배)·4kHz(위상 문제라 dt 무관) — 전부 주범 아님.

---

## 6. 핵심 파일

| 파일 | 내용 |
|---|---|
| `AirLib/include/vehicles/multirotor/MultiRotorParams.hpp` | `setupFrameEFTZ30` |
| `AirLib/include/common/AirSimSettings.hpp` | `Model` 필드 파싱 |
| `x_memory2/apps/SIL_App.cpp` | 프로파일·파라미터로딩·pid/reset·PID |
| `x_memory2/apps/SendCmd.cpp` | pid/reset 명령 |
| `x_memory2/build/pid_params_z30.txt` | 튜닝 파일 (gitignore) |
| `settings.json` / `Unreal/Environments/Blocks/settings.json` | `Model: EFTZ30`, PhysicsLoopPeriod |

백업 태그: `backup-pre-agri-frame` (프레임 작업 전 상태).
커밋: `8115d85e` feat: EFT Z30 농업용 드론 프레임 + 튜닝 가능한 비행제어.
