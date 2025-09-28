#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include "PhysicsEngine/BodyInstance.h"
#include "xlabSim.generated.h"
namespace msr { namespace airlib { class MultirotorRpcLibClient; } }
class USphereComponent;
class USceneComponent;
class APawn;
class UPrimitiveComponent;

#define __xlog(fmt, ...) \
do { \
  UE_LOG(LogTemp, Display, TEXT("### %s"), *FString::Printf(TEXT(fmt), ##__VA_ARGS__)); \
} while (0)

#define __xlogC(color, seconds, fmt, ...) \
do { \
  UE_LOG(LogTemp, Display, TEXT("### %s"), *FString::Printf(TEXT(fmt), ##__VA_ARGS__)); \
} while (0)

UCLASS()
class AIRSIM_API AxlabSim : public AActor
{
    GENERATED_BODY()

public:
    AxlabSim();
    virtual ~AxlabSim() override;

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public:
    virtual void Tick(float DeltaTime) override;

    void loop();
    void Takeoff();
    void Yawing();
    UFUNCTION(BlueprintCallable, Category = "xlabSim Controls")
    void StopCommands();
    UFUNCTION(BlueprintCallable, Category = "xlabSim Controls")
    void StartYawing(float YawRateDegPerSec = 180.0f);
    UFUNCTION(BlueprintCallable, Category = "xlabSim Controls")
    float GetYawDeg() const;
    void Arming();
    static constexpr int32 BuildNumber = 30;


    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "xlabSim Settings")
    FString VehicleName = TEXT("SimpleFlight");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "xlabSim Settings")
    int32 ScenarioVersion = 1;

private:
    enum class EFlightPhase
    {
        None,
        Arming,
        Takeoff,
        Yawing,
        Land,
        Done
    };

    struct{
        msr::airlib::MultirotorRpcLibClient* rpc = nullptr;
            std::atomic<bool> rpcReady{false};

        struct{
            EFlightPhase status = EFlightPhase::None;
            bool isCommandIssued = false;
        }phase;        

        struct{
            float lastTime = 0.0f;
            float nextSecs = 0.0f;
        }time;

        float startTime = 0.0f;

        struct{
            bool isYawing = false;
            float yawRateDegPerSec = 0.0f;
            float holdZ = 0.0f;
        }controls;
        
        struct{
            float lastLogTime = 0.0f;
            float intervalSecs = 1.0f;
            float lastYawDeg = 0.0f;
        }yaw;
        
        struct{
            std::atomic<bool> stop{false};
            std::atomic<long long> value{0};
            float accum = 0.0f;
            std::atomic<long long>  changed = 0;
        }counter;

        struct{
            FVector linearVelocityWS = FVector::ZeroVector;
            FVector linearAccelerationWS = FVector::ZeroVector;
            FVector angularVelocityRad = FVector::ZeroVector;
            FQuat orientation = FQuat::Identity;
            FVector lastLinearVelocityWS = FVector::ZeroVector;
            bool hasLastVelocity = false;
            float timestamp = 0.0f;
            bool valid = false;
            float yawDeg = 0.0f;
        }imu;

        struct{
            FCalculateCustomPhysics customPhysicsDelegate;
            class UPrimitiveComponent* component = nullptr;
            std::atomic<long long> calls{0};
            bool bound = false;
            long long lastCallsSnapshot = 0;
            std::atomic<int> simulate{0};
        }physics;
    }_m;
    
    std::thread _counterThread;
    std::mutex _counterMutex;
    std::condition_variable _counterCv;
    UPROPERTY()
    USphereComponent* PhysicsAnchor = nullptr;
    UPROPERTY()
    USceneComponent* SceneRoot = nullptr;

private:
    void StartCounterThread();
    void StopCounterThread();
    void UpdateImuFromPhysics(float DeltaTime);
    void OnCalculateCustomPhysics(float DeltaTime, struct FBodyInstance* BodyInstance);
    void TryBindPhysicsDelegate();
    void FindTargetPawn();
    UPrimitiveComponent* FindSimulatingPrimitive(AActor* Actor) const;

    UPROPERTY()
    APawn* TargetPawn = nullptr;
};
