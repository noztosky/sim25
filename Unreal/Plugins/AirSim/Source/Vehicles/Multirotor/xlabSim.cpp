#include "xlabSim.h"
#include "Engine/Engine.h"
#include "Async/Async.h"
#include "Misc/DateTime.h"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/VectorMath.hpp"

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
    StartCounterThread();
}

void AxlabSim::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    __xlog("xlabSim destroyed");
    StopCounterThread();
    if (_m.rpc)
    {
        delete _m.rpc;
        _m.rpc = nullptr;
    }
    Super::EndPlay(EndPlayReason);
}

void AxlabSim::StartCounterThread()
{
    _m.counter.stop = false;
    _m.counter.value = 0;
    _m.counter.accum = 0.0f;
    _counterThread = std::thread([this]() 
    {
        using clock = std::chrono::steady_clock;
        auto next_tick = clock::now() + std::chrono::milliseconds(1);
        auto next_log = clock::now() + std::chrono::seconds(1);
        while (!_m.counter.stop.load())
        {
            {
                std::unique_lock<std::mutex> lk(_counterMutex);
                _counterCv.wait_until(lk, next_tick, [this](){ return _m.counter.stop.load(); });
            }
            if (_m.counter.stop.load()) break;
            _m.counter.value.fetch_add(1);
            next_tick += std::chrono::milliseconds(1);

            auto now = clock::now();
            if (now >= next_log)
            {
                __xlog("counter=%lld, changed=%d", _m.counter.value.load(), _m.counter.changed.load());
                _m.counter.value.store(0);
                _m.counter.changed.store(0);
                do { next_log += std::chrono::seconds(1); } while (now >= next_log);
            }

            
            float yawDeg = GetYawDeg();
            if (_m.yaw.lastYawDeg != yawDeg)
            {
                _m.counter.changed.fetch_add(1);
            }
            _m.yaw.lastYawDeg = yawDeg;

            // no RPC calls in background thread
        }
    });
}

void AxlabSim::StopCounterThread()
{
    _m.counter.stop = true;
    _counterCv.notify_all();
    if (_counterThread.joinable())
    {
        _counterThread.join();
    }
}

void AxlabSim::Arming()
{
    const char* name = VehicleName.IsEmpty() ? "" : TCHAR_TO_ANSI(*VehicleName);
    _m.rpc->enableApiControl(true, name);
    if (!_m.rpc->isApiControlEnabled(name)) { __xlog("API disabled"); return; }
        bool armed = _m.rpc->armDisarm(true, name);
}

void AxlabSim::Takeoff()
{
    const char* name = VehicleName.IsEmpty() ? "" : TCHAR_TO_ANSI(*VehicleName);
    _m.rpc->enableApiControl(true, name);
    _m.rpc->takeoffAsync(2.0f, name);
}

void AxlabSim::Yawing()
{
    //Async(EAsyncExecution::ThreadPool, [this]() {
        try {
            const char* name = VehicleName.IsEmpty() ? "" : TCHAR_TO_ANSI(*VehicleName);
            _m.rpc->rotateByYawRateAsync(180.0f, 0.2f, name);
        } catch (...) {
            __xlog("rpc exception");
        }
    //});
}

void AxlabSim::StartYawing(float YawRateDegPerSec)
{
    //Async(EAsyncExecution::ThreadPool, [this, YawRateDegPerSec]() {
        try {
            const char* name = VehicleName.IsEmpty() ? "" : TCHAR_TO_ANSI(*VehicleName);
            _m.controls.isYawing = true;
            _m.controls.yawRateDegPerSec = YawRateDegPerSec;
            _m.rpc->rotateByYawRateAsync(YawRateDegPerSec, 0.2f, name);
            __xlogC(FColor::Blue, 2.0f, "StartYawing: %.1f deg/s", YawRateDegPerSec);
        } catch (...) {
            __xlog("rpc exception");
        }
    //});
}

float AxlabSim::GetYawDeg() const
{
    if (!_m.rpc)
        return 0.0f;
    try
    {
        const char* name = VehicleName.IsEmpty() ? "" : TCHAR_TO_ANSI(*VehicleName);
        const auto state = _m.rpc->getMultirotorState(name);
        const auto& q = state.kinematics_estimated.pose.orientation;
        float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
        msr::airlib::VectorMath::toEulerianAngle(q, pitch, roll, yaw);
        return FMath::RadiansToDegrees(yaw);
    }
    catch (...)
    {
        return 0.0f;
    }
}

void AxlabSim::StopCommands()
{
    Async(EAsyncExecution::ThreadPool, [this]() {
        try {
            const char* name = VehicleName.IsEmpty() ? "" : TCHAR_TO_ANSI(*VehicleName);
            // cancel currently running long task on server first
            _m.rpc->cancelLastTask(name);
            _m.rpc->waitOnLastTask(nullptr, 1.0f);
            // then issue a hover to stabilize and zero yaw rate
            _m.rpc->hoverAsync(name);
            _m.rpc->waitOnLastTask(nullptr, 1.0f);
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
            case EFlightPhase::Arming:
                _m.phase.status = EFlightPhase::Takeoff;
                _m.time.nextSecs = 1.0f;
                __xlogC(FColor::Blue, 1.0f, "Arming");
                 Arming();
                break;
            case EFlightPhase::Takeoff:
                _m.time.nextSecs = 3.0f;
                _m.phase.status = EFlightPhase::Yawing;
                __xlogC(FColor::Blue, 1.0f, "Takeoff");
                Takeoff();
                break;
            case EFlightPhase::Yawing:
                _m.time.nextSecs = 0.2f;
                _m.phase.status = EFlightPhase::Yawing;
                //__xlogC(FColor::Blue, 1.0f, "Yawing");
                Yawing();
                break;
        }
        _m.phase.isCommandIssued = true;
    }
    
}
