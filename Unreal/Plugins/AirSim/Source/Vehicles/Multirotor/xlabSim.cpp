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

    // PWM reader init (external x_memory path)
    static msr::airlib::XlabXMemoryReader pwm_reader;
    _pwmReaderPtr = &pwm_reader;
    const std::string default_path = std::string("D:\\open\\x_memory\\pwm.bin");
    _pwmReaderPtr->initialize(default_path);
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
    // Read PWM from external memory and feed API if available
    if (_pwmReaderPtr) {
        float pwm[8] = {};
        int count = 0;
        if (_pwmReaderPtr->read(pwm, 8, &count) && count >= 4) {
            msr::airlib::MultirotorApiBase* api = ResolveApi(this);
            if (api) {
                // PWM 배열 순서는 사용자 약속에 맞게 조정 필요
                api->moveByMotorPWMs(pwm[0], pwm[1], pwm[2], pwm[3], 0.02f);
            }
        }
    }

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