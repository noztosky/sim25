#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "xlabSim.generated.h"
namespace msr { namespace airlib { class MultirotorRpcLibClient; } }

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


    static constexpr int32 BuildNumber = 19;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "xlabSim Settings")
    FString VehicleName = TEXT("SimpleFlight");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "xlabSim Settings")
    int32 ScenarioVersion = 1;

private:
    enum class EFlightPhase
    {
        None,
        Takeoff,
            Yawing,
        Land,
        Done
    };

    struct{
        msr::airlib::MultirotorRpcLibClient* rpc = nullptr;

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
    }_m;
};
