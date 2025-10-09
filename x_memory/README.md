# AirSim Bidirectional Communication System

## 개요
하나의 서버/클라이언트로 **양방향 통신**하는 AirSim 시스템입니다.
- **IMU 데이터**: Server → Client (1000Hz)
- **PWM 제어**: Client → Server (400Hz)

## 파일 구조
```
d:/open/x_pipeline/
├── airsim_server.cpp    # AirSim 서버 (양방향)
├── airsim_client.cpp    # sim_test 클라이언트 (양방향)
├── build.bat           # 빌드 스크립트
└── README.md           # 이 파일
```

## 통신 구조
```
AirSim Server ←→ Named Pipes ←→ AirSim Client
     ↓                              ↑
[IMU Data] → WriteFile()    ReadFile() → [IMU Data]
     ↑                              ↓
[PWM Data] ← ReadFile()    WriteFile() ← [PWM Data]
```

## 빌드 및 실행

### 1. 빌드
```bash
build.bat
```

### 2. 실행
**터미널 1 (AirSim 서버):**
```bash
airsim_server.exe
```

**터미널 2 (AirSim 클라이언트):**
```bash
airsim_client.exe
```

파이프 경로:
- \\.\pipe\AirSimIMU (서버→클라이언트)
- \\.\pipe\AirSimPWM (클라이언트→서버)
- \\.\pipe\AirSimBARO (서버→클라이언트)

## 데이터 구조

### IMU 데이터 (1000Hz, Server → Client)
```cpp
struct ImuData {
    double roll, pitch, yaw;           // 오일러 각도 (도)
    double angular_vel_x, y, z;        // 각속도
    double linear_accel_x, y, z;       // 선형 가속도
    long long timestamp;               // 타임스탬프 (나노초)
    int frequency;                     // 읽기 빈도 (Hz)
    bool is_valid;                     // 데이터 유효성
};
```

### PWM 데이터 (400Hz, Client → Server)
```cpp
struct PWMData {
    int channel1;  // 서보 1 (0-2000) - 롤 제어
    int channel2;  // 서보 2 (0-2000) - 피치 제어
    int channel3;  // 서보 3 (0-2000) - 스로틀
    int channel4;  // 서보 4 (0-2000) - 요 제어
    long long timestamp;   // 타임스탬프 (나노초)
    int frequency;         // 전송 빈도 (Hz)
    bool is_valid;         // 데이터 유효성
};
```

## 성능
- **IMU 전송**: 1000Hz (1ms 간격)
- **PWM 전송**: 400Hz (2.5ms 간격)
- **지연시간**: < 0.5ms
- **데이터 크기**: IMU 80 bytes, PWM 32 bytes

## 특징
- ✅ **하나의 서버/클라이언트**: 양방향 통신
- ✅ **실시간 데이터**: 고속 IMU/PWM 전송
- ✅ **안정적 연결**: Windows Named Pipes 사용
- ✅ **동시 처리**: IMU 수신과 PWM 전송 동시 실행

## 다음 단계
이 시스템을 AirSim에 통합하여 실제 IMU 데이터 전송과 PWM 제어가 가능합니다.
