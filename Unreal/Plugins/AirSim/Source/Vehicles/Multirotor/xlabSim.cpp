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
    Async(EAsyncExecution::ThreadPool, [this]() {
        const char* name = VehicleName.IsEmpty() ? "" : TCHAR_TO_ANSI(*VehicleName);
        try {
            _m.rpc->confirmConnection();
            _m.rpc->simPause(false);
            _m.rpc->enableApiControl(true, name);
            if (!_m.rpc->isApiControlEnabled(name)) { __xlog("API disabled"); return; }
            bool armed = _m.rpc->armDisarm(true, name);
            __xlog("armed=%d", (int)armed);
            _m.rpc->takeoffAsync(20.0f, name);
            auto st = _m.rpc->getMultirotorState(name);
            __xlog("landed=%d z=%.2f", (int)st.landed_state, st.kinematics_estimated.pose.position.z());
        } catch (...) {
            __xlog("rpc exception");
        }
    });
}

void AxlabSim::Tick(float DeltaTime) {
    Super::Tick(DeltaTime);
    
    if (_m.phase.isCommandIssued == false)
    {
        switch (_m.phase.status)
        {
            case EFlightPhase::None:
                _m.phase.status = EFlightPhase::Takeoff;
                 Takeoff();
                 __xlogC(FColor::Blue, 3.0f, "Takeoff command issued");
                break;
            case EFlightPhase::Takeoff:
                break;
        }

        _m.phase.isCommandIssued = true;
    }
}
