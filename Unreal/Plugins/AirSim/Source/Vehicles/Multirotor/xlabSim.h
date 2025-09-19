#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "xlabSim.generated.h"
namespace msr { namespace airlib { class MultirotorRpcLibClient; } }

#define __xlog(fmt, ...) \
do { \
  UE_LOG(LogTemp, Log, TEXT(fmt), ##__VA_ARGS__); \
  if (GEngine) { \
    GEngine->AddOnScreenDebugMessage( \
      -1, 3.0f, FColor::Blue, FString::Printf(TEXT(fmt), ##__VA_ARGS__)); \
  } \
} while (0)

#define __xlogC(color, seconds, fmt, ...) \
do { \
  UE_LOG(LogTemp, Log, TEXT(fmt), ##__VA_ARGS__); \
  if (GEngine) { \
    GEngine->AddOnScreenDebugMessage( \
      -1, seconds, color, FString::Printf(TEXT(fmt), ##__VA_ARGS__)); \
  } \
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


    static constexpr int32 BuildNumber = 7;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "xlabSim Settings")
    FString VehicleName = TEXT("SimpleFlight");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "xlabSim Settings")
    int32 ScenarioVersion = 1;

private:

    enum class EFlightPhase
    {
        None,
        Takeoff,
        Land,
        Done
    };

    struct{
        msr::airlib::MultirotorRpcLibClient* rpc = nullptr;

        struct{
            EFlightPhase status = EFlightPhase::None;
            bool isCommandIssued = false;
        }phase;
        

        float startTime = 0.0f;
    }_m;
};
