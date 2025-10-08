// includes must come first
#include "xlabSim.h"
#include "Engine/Engine.h"
#include "Async/Async.h"
#include "Misc/DateTime.h"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include "SimMode/SimModeBase.h"
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

    UPrimitiveComponent* target = nullptr;
    if (TargetPawn)
        target = FindSimulatingPrimitive(TargetPawn);
    if (!target)
        target = Cast<UPrimitiveComponent>(GetRootComponent());
    if (!target)
        return;

    _m.physics.component = target;
    if (!target->BodyInstance.bSimulatePhysics)
    {
        __xlog("enabling simulate physics on %s", *target->GetName());
        target->SetSimulatePhysics(true);
    }
    target->BodyInstance.AddCustomPhysics(_m.physics.customPhysicsDelegate);
    _m.physics.bound = true;
    _m.physics.simulate.store(target->BodyInstance.bSimulatePhysics ? 1 : 0);
    __xlog("physics (re)bound: %d simulate=%d on %s", (int)_m.physics.bound, _m.physics.simulate.load(), *target->GetName());
}

// Resolve in-process Multirotor API (no RPC network)
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

UPrimitiveComponent* AxlabSim::FindSimulatingPrimitive(AActor* Actor) const
{
    if (!Actor) return nullptr;
    TArray<UActorComponent*> comps;
    Actor->GetComponents(UPrimitiveComponent::StaticClass(), comps);
    for (UActorComponent* c : comps)
    {
        if (UPrimitiveComponent* p = Cast<UPrimitiveComponent>(c))
        {
            if (p->IsSimulatingPhysics()) return p;
        }
    }
    return nullptr;
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
            float yawDeg = GetYawDeg();
            
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
                __xlogC(FColor::Green, 1.5f, "counter=%lld, changed=%d, lastYawDeg=%.1f, physics_calls=%lld bound=%d sim=%d"
                    , _m.counter.value.load(), _m.counter.changed.load(), yawDeg
                    , calls, (int)_m.physics.bound, sim);
                UAirBlueprintLib::LogMessage(
                    FString::Printf(TEXT("counter=%lld, changed=%d, lastYawDeg=%.1f, physics_calls=%lld bound=%d sim=%d"),
                        _m.counter.value.load(), _m.counter.changed.load(), yawDeg,
                        calls, (int)_m.physics.bound, sim),
                    TEXT(""), LogDebugLevel::Informational, 1.5f);
                _m.counter.value.store(0);
                _m.counter.changed.store(0);
                do { next_log += std::chrono::seconds(1); } while (now >= next_log);
            }

            
            //float yawDeg = _m.imu.yawDeg.load();
            
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
    try {
        auto* api = ResolveApi(this);
        if (!api) { __xlog("api null"); return; }
        api->enableApiControl(true);
        bool armed = api->armDisarm(true);
        __xlog("armed=%d", (int)armed);
    } catch (...) {
        __xlog("api exception in Arming");
    }
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

float AxlabSim::GetYawDeg() const
{
    if (_m.imu.valid.load())
    {
        std::lock_guard<std::mutex> lk(_imuMutex);
        const float yaw_from_quat = FMath::UnwindDegrees(_m.imu.orientation.Rotator().Yaw);
        return yaw_from_quat;
    }
    const USceneComponent* root = GetRootComponent();
    if (root)
        return FMath::UnwindDegrees(root->GetComponentRotation().Yaw);
    return 0.0f;
}

void AxlabSim::StopCommands()
{
    try {
        auto* api = ResolveApi(this);
        if (!api) { __xlog("api null"); return; }
        api->cancelLastTask();
        api->hover();
        _m.phase.status = EFlightPhase::Done;
        _m.phase.isCommandIssued = true;
        __xlogC(FColor::Red, 2.0f, "Emergency stop: hover issued");
    } catch (...) {
        __xlog("api exception");
    }
}

void AxlabSim::Tick(float DeltaTime) {
    Super::Tick(DeltaTime);

    if (!_m.physics.bound || (_m.physics.calls.load() == _m.physics.lastCallsSnapshot))
    {
        TryBindPhysicsDelegate();
    }
    if (_m.physics.component)
    {
        _m.physics.component->BodyInstance.AddCustomPhysics(_m.physics.customPhysicsDelegate);
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

    // removed RPC-based continuous yawing during tick to avoid blocking/stalls
    
}