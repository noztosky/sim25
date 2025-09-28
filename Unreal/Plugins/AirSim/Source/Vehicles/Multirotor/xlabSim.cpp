// includes must come first
#include "xlabSim.h"
#include "Engine/Engine.h"
#include "Async/Async.h"
#include "Misc/DateTime.h"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/VectorMath.hpp"
#include "Components/PrimitiveComponent.h"
#include "PhysicsEngine/BodyInstance.h"
#include "Components/SphereComponent.h"
#include "Components/SceneComponent.h"
#include "Kismet/GameplayStatics.h"

void AxlabSim::TryBindPhysicsDelegate()
{
    if (_m.physics.bound && _m.physics.component && _m.physics.component->BodyInstance.bSimulatePhysics)
        return;

    if (UPrimitiveComponent* root = Cast<UPrimitiveComponent>(GetRootComponent()))
    {
        _m.physics.component = root;
        if (!root->BodyInstance.bSimulatePhysics)
        {
            __xlog("enabling simulate physics on %s", *root->GetName());
            root->SetSimulatePhysics(true);
        }
        _m.physics.customPhysicsDelegate = FCalculateCustomPhysics::CreateUObject(this, &AxlabSim::OnCalculateCustomPhysics);
        root->BodyInstance.AddCustomPhysics(_m.physics.customPhysicsDelegate);
        _m.physics.bound = true;
        _m.physics.simulate.store(root->BodyInstance.bSimulatePhysics ? 1 : 0);
        __xlog("physics (re)bound: %d simulate=%d", (int)_m.physics.bound, _m.physics.simulate.load());
    }
}

void AxlabSim::FindTargetPawn()
{
    UWorld* world = GetWorld();
    if (!world)
        return;
    TArray<AActor*> actors;
    UGameplayStatics::GetAllActorsOfClass(world, APawn::StaticClass(), actors);
    if (actors.Num() > 0)
    {
        TargetPawn = Cast<APawn>(actors[0]);
        __xlog("TargetPawn set: %s", *GetNameSafe(TargetPawn));
    }
}

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

    // Ensure we have a primitive root with simulate physics
    SceneRoot = GetRootComponent();
    if (!Cast<UPrimitiveComponent>(SceneRoot))
    {
        PhysicsAnchor = NewObject<USphereComponent>(this, TEXT("PhysicsAnchor"));
        PhysicsAnchor->InitSphereRadius(10.0f);
        PhysicsAnchor->SetSimulatePhysics(true);
        PhysicsAnchor->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
        PhysicsAnchor->SetCollisionProfileName(UCollisionProfile::PhysicsActor_ProfileName);
        PhysicsAnchor->RegisterComponent();
        SetRootComponent(PhysicsAnchor);
    }

    TryBindPhysicsDelegate();
    FindTargetPawn();
    StartCounterThread();
    return;
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
                const long long calls = _m.physics.calls.load();
                const int sim = _m.physics.simulate.load();
                __xlog("counter=%lld, changed=%d, lastYawDeg=%.1f, physics_calls=%lld bound=%d sim=%d"
                    , _m.counter.value.load(), _m.counter.changed.load(), _m.yaw.lastYawDeg
                    , calls, (int)_m.physics.bound, sim);
                _m.counter.value.store(0);
                _m.counter.changed.store(0);
                do { next_log += std::chrono::seconds(1); } while (now >= next_log);
            }

            
            float yawDeg = _m.imu.yawDeg;
            if (_m.yaw.lastYawDeg != yawDeg)
            {
                _m.counter.changed.fetch_add(1);
            }
            _m.yaw.lastYawDeg = yawDeg;

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
    if (_m.imu.valid)
        return _m.imu.yawDeg;
    const USceneComponent* root = GetRootComponent();
    if (root)
        return FMath::UnwindDegrees(root->GetComponentRotation().Yaw);
    return 0.0f;
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

    if (!_m.physics.bound || (_m.physics.calls.load() == _m.physics.lastCallsSnapshot))
    {
        TryBindPhysicsDelegate();
    }
    _m.physics.lastCallsSnapshot = _m.physics.calls.load();
    if (!TargetPawn)
    {
        FindTargetPawn();
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
                __xlogC(FColor::Blue, 1.0f, "Yawing");
                Yawing();
                break;
        }
        _m.phase.isCommandIssued = true;
    }

    if (_m.rpc && _m.rpcReady.load() && _m.phase.status == EFlightPhase::Yawing)
    {
        const char* name = VehicleName.IsEmpty() ? "" : TCHAR_TO_ANSI(*VehicleName);
        _m.rpc->rotateByYawRateAsync(180.0f, DeltaTime, name);
    }
    
}

void AxlabSim::OnCalculateCustomPhysics(float DeltaTime, FBodyInstance* BodyInstance)
{
    if (!BodyInstance)
        return;

    _m.imu.valid = false;
    _m.physics.calls.fetch_add(1);

    FTransform transform = BodyInstance->GetUnrealWorldTransform();
    if (TargetPawn)
    {
        transform = TargetPawn->GetActorTransform();
    }
    const FVector velocity = BodyInstance->GetUnrealWorldVelocity();
    const FVector angular = BodyInstance->GetUnrealWorldAngularVelocityInRadians();

    _m.imu.orientation = transform.GetRotation();
    _m.imu.yawDeg = FMath::UnwindDegrees(_m.imu.orientation.Rotator().Yaw);
    _m.imu.linearVelocityWS = velocity;
    if (_m.imu.hasLastVelocity && DeltaTime > KINDA_SMALL_NUMBER)
    {
        _m.imu.linearAccelerationWS = (velocity - _m.imu.lastLinearVelocityWS) / DeltaTime;
    }
    _m.imu.lastLinearVelocityWS = velocity;
    _m.imu.hasLastVelocity = true;
    _m.imu.angularVelocityRad = angular;
    _m.imu.yawDeg = FMath::UnwindDegrees(_m.imu.orientation.Rotator().Yaw);

    if (UWorld* world = GetWorld())
    {
        _m.imu.timestamp = world->GetTimeSeconds();
    }
    _m.imu.valid = true;
}
