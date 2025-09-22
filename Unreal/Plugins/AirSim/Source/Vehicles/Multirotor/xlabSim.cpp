#include "xlabSim.h"
#include "Engine/Engine.h"
#include "Async/Async.h"
#include "Misc/DateTime.h"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

AxlabSim::AxlabSim() {
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bStartWithTickEnabled = true;
}

AxlabSim::~AxlabSim() {}

void AxlabSim::BeginPlay()
{
    Super::BeginPlay();
    SetActorTickEnabled(true);
    __xlogC(FColor::Cyan, 5.0f, "xlabSim Build #%d", BuildNumber);

    _m.rpc = new msr::airlib::MultirotorRpcLibClient();
    _m.phase.status = EFlightPhase::None;
    _m.phase.isCommandIssued = false;
    _m.startTime = static_cast<float>(FDateTime::UtcNow().ToUnixTimestamp());
    __xlogC(FColor::Blue, 3.0f, "xlabSim initialized");
}

void AxlabSim::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    __xlog("xlabSim destroyed");
    if (_m.rpc)
    {
        delete _m.rpc;
        _m.rpc = nullptr;
    }
    Super::EndPlay(EndPlayReason);
}

void AxlabSim::Takeoff()
{
    //*
    Async(EAsyncExecution::ThreadPool, [this]() {
        try {
            const char* name = VehicleName.IsEmpty() ? "" : TCHAR_TO_ANSI(*VehicleName);
            _m.rpc->enableApiControl(true, name);
            if (!_m.rpc->isApiControlEnabled(name)) { __xlog("API disabled"); return; }
            bool armed = _m.rpc->armDisarm(true, name);
            __xlog("armed=%d", (int)armed);
            _m.rpc->takeoffAsync(20.0f, name);
            //_m.rpc->rotateByYawRateAsync(180.0f, 2.0f, name);
        } catch (...) {
            __xlog("rpc exception");
        }
    });
    /*/
    //const char* name = VehicleName.IsEmpty() ? "" : TCHAR_TO_ANSI(*VehicleName);
    //_m.rpc->enableApiControl(true, name);
    if (!_m.rpc->isApiControlEnabled(name)) { __xlog("API disabled"); return; }
    bool armed = _m.rpc->armDisarm(true, name);
    __xlog("armed=%d", (int)armed);
    _m.rpc->takeoff(20.0f, name);
    auto st = _m.rpc->getMultirotorState(name);
    //*/
}

void AxlabSim::Yawing()
{
    Async(EAsyncExecution::ThreadPool, [this]() {
        try {
            const char* name = VehicleName.IsEmpty() ? "" : TCHAR_TO_ANSI(*VehicleName);
            _m.rpc->rotateByYawRateAsync(180.0f, 20.0f, name);
        } catch (...) {
            __xlog("rpc exception");
        }
    });
}

void AxlabSim::StopCommands()
{
    Async(EAsyncExecution::ThreadPool, [this]() {
        try {
            const char* name = VehicleName.IsEmpty() ? "" : TCHAR_TO_ANSI(*VehicleName);
            // cancel currently running long task on server first
            _m.rpc->cancelLastTask(name);
            // then issue a hover to stabilize and zero yaw rate
            _m.rpc->hoverAsync(name);
            _m.phase.status = EFlightPhase::Done;
            _m.phase.isCommandIssued = true;
            __xlogC(FColor::Red, 2.0f, "Emergency stop: hover issued");
        } catch (...) {
            __xlog("rpc exception");
        }
    });
}

void AxlabSim::Tick(float DeltaTime) {
    Super::Tick(DeltaTime);

    float currentTime = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f;
    float deltaTime = currentTime - _m.time.lastTime;
    if(deltaTime >= _m.time.nextSecs)
    {
        _m.time.lastTime = currentTime;
        _m.phase.isCommandIssued = false;
    }
    
    if (_m.phase.isCommandIssued == false)
    {
        switch (_m.phase.status)
        {
            case EFlightPhase::None:
                _m.phase.status = EFlightPhase::Takeoff;
                _m.time.nextSecs = 3.0f;
                 Takeoff();
                 __xlogC(FColor::Blue, 3.0f, "Takeoff command issued");
                break;
            case EFlightPhase::Takeoff:
                Yawing();
                _m.time.nextSecs = 20.0f;
                __xlogC(FColor::Blue, 3.0f, "Yawing command issued");
                break;
        }
        _m.phase.isCommandIssued = true;
    }
}
