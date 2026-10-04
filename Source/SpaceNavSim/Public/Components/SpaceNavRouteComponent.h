#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Game/Pawn/SpaceNavRouteOptimizationDataAsset.h"
#include "SpaceNavRouteComponent.generated.h"

class USpaceNavPawnSettingsDataAsset;
class USpaceNavEngineComponent;
class USpaceNavResourceStoreComponent;
class ULineBatchComponent;
class AActor;
class ASpaceNavPawn;
struct FRouteSearchResults;
struct FRouteAutopilotState;

UCLASS(BlueprintType, Blueprintable, ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class SPACENAVSIM_API USpaceNavRouteComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	USpaceNavRouteComponent();

	UFUNCTION(BlueprintCallable, Category = "SpaceNavRoute|Route")
	void BuildFuelEfficientRoute();

	UFUNCTION(BlueprintCallable, Category = "SpaceNavRoute|Route")
	void BuildTimeEfficientRoute();

	UFUNCTION(BlueprintCallable, Category = "SpaceNavRoute|Route")
	void BuildBalancedRoute();

	UFUNCTION(BlueprintCallable, Category = "SpaceNavRoute|Route")
	void MoveRoad();

	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	UPROPERTY(EditAnywhere, Category = "Settings|Data")
	TObjectPtr<USpaceNavPawnSettingsDataAsset> PawnSettings;

	UPROPERTY(EditAnywhere, Category = "Settings|Data")
	TObjectPtr<USpaceNavRouteOptimizationDataAsset> OptimizationSettings;

private:
	enum class ERouteMode : uint8
	{
		Fuel,
		Time,
		Balanced
	};

	void BuildRouteForMode(ERouteMode Mode);
	void StartBackgroundSearch();
	void OnSearchCompleted(TSharedPtr<FRouteSearchResults, ESPMode::ThreadSafe> Results);
	bool CheckSettings() const;
	bool FindTargetLocation(FVector& OutTargetLocation) const;
	bool CollectObstacles(TArray<FSphere>& OutObstacles, int32& OutStartPlanetIndex,
		double& OutStartPlanetRadius) const;
	bool BuildStartEscape(const FVector& TargetLocation, const TArray<FSphere>& Obstacles,
		int32 StartPlanetIndex, double StartPlanetRadius, TArray<FVector>& OutEscapePoints) const;
	void DrawRoute();
	void AdvanceMoveRoad(float DeltaTime);
	void ExecuteScriptedFlight(float DeltaTime, ASpaceNavPawn& Pawn,
		USpaceNavEngineComponent& Engine, USpaceNavResourceStoreComponent& Resources);
	bool AdvanceScriptedSegments(double EndSeconds, float FrameDeltaTime, ASpaceNavPawn& Pawn,
		USpaceNavEngineComponent& Engine, USpaceNavResourceStoreComponent& Resources);
	void UpdateScriptedPose(ASpaceNavPawn& Pawn, USpaceNavEngineComponent& Engine,
		USpaceNavResourceStoreComponent& Resources);
	void FinishAutopilot(const TCHAR* Result);
	void CaptureCollisionBodies();
	void CheckRouteCollision(const FVector& PawnLocation, float DeltaTime,
		double FrameFraction, const TArray<FVector>& FrameStartBodyPositions,
		const TArray<FVector>& FrameEndBodyPositions);
	void ResumeEngineAfterSearch();
	void LogDisplay(const TCHAR* Message) const;

	struct FMonitoredBody
	{
		TWeakObjectPtr<AActor> Actor;
		FVector PreviousLocation = FVector::ZeroVector;
		double RadiusUU = 0.0;
		bool bStar = false;
	};

	TArray<FVector> GuidancePoints;
	TArray<FVector> RoutePoints;
	TArray<FMonitoredBody> MonitoredBodies;
	TWeakObjectPtr<USpaceNavEngineComponent> PausedEngine;
	UPROPERTY(Transient)
	TObjectPtr<ULineBatchComponent> RouteLineBatch;
	int32 NextRoutePointIndex = INDEX_NONE;
	int32 SelectedCandidateIndex = INDEX_NONE;
	int32 ModeCandidateIndices[3] = {INDEX_NONE, INDEX_NONE, INDEX_NONE};
	double RouteElapsedSeconds = 0.0;
	FVector PreviousPawnLocation = FVector::ZeroVector;
	bool bHasPreviousPawnLocation = false;
	bool bImpactLogged = false;
	bool bEnginePausedForSearch = false;
	bool bEngineTickWasEnabled = false;
	bool bRouteReady = false;
	bool bLogRoute = true;
	bool bSearchStarted = false;
	bool bSearchFinished = false;
	bool bSearchStopped = false;
	bool bHasPendingMode = false;
	ERouteMode PendingMode = ERouteMode::Fuel;
	TSharedPtr<FRouteSearchResults, ESPMode::ThreadSafe> RoutePool;
	TSharedPtr<FRouteAutopilotState> AutopilotState;
};
