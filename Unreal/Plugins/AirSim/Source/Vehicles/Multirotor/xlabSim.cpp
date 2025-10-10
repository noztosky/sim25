// includes must come first
#include "xlabSim.h"
#include "Engine/Engine.h"
#include "Async/Async.h"
#include "Misc/DateTime.h"
#include "SimMode/SimModeBase.h"
#include "common/VectorMath.hpp"
#include "common/XlabXMemoryAdapter.hpp"

// physics delegate removed

static msr::airlib::MultirotorApiBase* ResolveApi(const AxlabSim* self)
{
    ASimModeBase* sim = ASimModeBase::getSimMode();
    if (!sim) return nullptr;
    auto* provider = sim->getApiProvider();
    if (!provider) return nullptr;
    const char* name = self->VehicleName.IsEmpty() ? "" : TCHAR_TO_ANSI(*self->VehicleName);
    auto* api = static_cast<msr::airlib::MultirotorApiBase*>(provider->getVehicleApi(name));
    if (!api) {
        api = static_cast<msr::airlib::MultirotorApiBase*>(provider->getVehicleApi(""));
    }
    return api;
}

// target pawn search removed

// sim primitive lookup removed

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

    _m.phase.status = EFlightPhase::None;
    _m.phase.isCommandIssued = false;
    _m.startTime = static_cast<float>(FDateTime::UtcNow().ToUnixTimestamp());
    __xlogC(FColor::Blue, 3.0f, "xlabSim initialized");

    return;
}

void AxlabSim::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    Super::EndPlay(EndPlayReason);
}

void AxlabSim::Arming()
{
    auto* api = ResolveApi(this);
    if (!api) { __xlog("api null"); return; }
    api->enableApiControl(true);
    bool armed = api->armDisarm(true);
    __xlog("armed=%d", (int)armed);
}

void AxlabSim::Takeoff()
{
    Async(EAsyncExecution::ThreadPool, [this]() {
        // Resolve API on GameThread to be safe
        msr::airlib::MultirotorApiBase* api = nullptr;
        FEvent* got = FPlatformProcess::GetSynchEventFromPool(true);
        AsyncTask(ENamedThreads::GameThread, [this, &api, got]() {
            api = ResolveApi(this);
            got->Trigger();
        });
        got->Wait();
        FPlatformProcess::ReturnSynchEventToPool(got);
        if (!api) { __xlog("api null"); return; }
        api->enableApiControl(true);
        api->takeoff(2.0f);
    });
}

void AxlabSim::Yawing()
{
    Async(EAsyncExecution::ThreadPool, [this]() {
        msr::airlib::MultirotorApiBase* api = nullptr;
        FEvent* got = FPlatformProcess::GetSynchEventFromPool(true);
        AsyncTask(ENamedThreads::GameThread, [this, &api, got]() {
            api = ResolveApi(this);
            got->Trigger();
        });
        got->Wait();
        FPlatformProcess::ReturnSynchEventToPool(got);
        if (!api) { __xlog("api null"); return; }
        api->rotateByYawRate(90.0f, 0.2f);
    });
}

// GetYawDeg removed; IMU handled elsewhere

void AxlabSim::StopCommands()
{
    auto* api = ResolveApi(this);
    if (!api) { __xlog("api null"); return; }
    api->cancelLastTask();
    api->hover();
    _m.phase.status = EFlightPhase::Done;
    _m.phase.isCommandIssued = true;
    __xlogC(FColor::Red, 2.0f, "Emergency stop: hover issued");   
}

void AxlabSim::Tick(float DeltaTime) {
    Super::Tick(DeltaTime);
    // external PWM handling moved to API layer

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
                _m.phase.status = EFlightPhase::Arming;
                _m.time.nextSecs = 3.0f;
                __xlogC(FColor::Blue, 1.0f, "None");
                break;
            case EFlightPhase::Arming:
                _m.phase.status = EFlightPhase::Takeoff;
                _m.time.nextSecs = 1.0f;
                __xlogC(FColor::Blue, 1.0f, "Arming");
                Arming();
                break;
            case EFlightPhase::Takeoff:
                _m.time.nextSecs = 3.0f;
                __xlogC(FColor::Blue, 1.0f, "Takeoff");
                Takeoff();
                // advance after issuing takeoff
                _m.phase.status = EFlightPhase::Yawing;
                break;
            case EFlightPhase::Yawing:
                _m.time.nextSecs = 0.2f;
                _m.phase.status = EFlightPhase::Yawing;
                Yawing();
                break;
        }
        _m.phase.isCommandIssued = true;
    }    
}