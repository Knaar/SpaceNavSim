#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "SpaceNavRouteComponent.generated.h"

class USpaceNavPawnSettingsDataAsset;

UCLASS(BlueprintType, Blueprintable, ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class SPACENAVSIM_API USpaceNavRouteComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UFUNCTION(BlueprintCallable, Category = "SpaceNavRoute|Route")
	void BuildRoute();

	UPROPERTY(EditAnywhere, Category = "Settings|Data")
	TObjectPtr<USpaceNavPawnSettingsDataAsset> PawnSettings;

private:
	bool CheckSettings() const;
	bool FindTargetLocation(FVector& OutTargetLocation) const;
	bool CollectObstacles(TArray<FSphere>& OutObstacles) const;
	bool ComputeRoute(const FVector& TargetLocation, const TArray<FSphere>& Obstacles);
	bool SmoothRoute(const TArray<FSphere>& Obstacles);
	void DrawRoute() const;

	void CreateNodes(const FVector& TargetLocation, const TArray<FSphere>& Obstacles,
		TArray<FVector>& OutNodes) const;
	bool FindShortestPath(const TArray<FVector>& Nodes, const TArray<FSphere>& Obstacles,
		TArray<int32>& OutPrevious) const;
	void BuildSmoothedPoints(const TArray<FVector>& Corners, double BlendFraction,
		TArray<FVector>& OutPoints) const;
	bool IsRouteClear(const TArray<FVector>& Points, const TArray<FSphere>& Obstacles) const;
	bool IsSegmentClear(const FVector& Start, const FVector& End,
		const TArray<FSphere>& Obstacles) const;
	void LogDisplay(const TCHAR* Message) const;

	TArray<FVector> RoutePoints;
	bool bLogRoute = true;
};
