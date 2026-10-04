#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "SpaceNavRouteOptimizationDataAsset.generated.h"

USTRUCT(BlueprintType)
struct FSpaceNavRouteWeightSet
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weights", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	double FuelWeight = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weights", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	double TimeWeight = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weights", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	double RiskWeight = 0.0;
};

UCLASS(BlueprintType)
class SPACENAVSIM_API USpaceNavRouteOptimizationDataAsset : public UDataAsset
{
	GENERATED_BODY()

public:
	USpaceNavRouteOptimizationDataAsset()
	{
		FuelEfficient.FuelWeight = 1.0;
		TimeEfficient.TimeWeight = 1.0;
		Balanced.FuelWeight = 0.6;
		Balanced.TimeWeight = 0.3;
		Balanced.RiskWeight = 0.1;
	}

	UPROPERTY(EditAnywhere, Category = "Settings|Fuel Efficient")
	FSpaceNavRouteWeightSet FuelEfficient;

	UPROPERTY(EditAnywhere, Category = "Settings|Time Efficient")
	FSpaceNavRouteWeightSet TimeEfficient;

	UPROPERTY(EditAnywhere, Category = "Settings|Balanced")
	FSpaceNavRouteWeightSet Balanced;

	UPROPERTY(EditAnywhere, Category = "Settings|Prediction", meta = (ClampMin = "0", ClampMax = "2"))
	int32 FuelMaxTurns = 2;

	UPROPERTY(EditAnywhere, Category = "Settings|Prediction", meta = (ClampMin = "0.01", ClampMax = "1.0"))
	TArray<double> CruiseSpeedFractions = {0.5, 0.75, 1.0};

	UPROPERTY(EditAnywhere, Category = "Settings|Prediction", meta = (ClampMin = "1"))
	int32 MaxGuideCandidates = 200;

	UPROPERTY(EditAnywhere, Category = "Settings|Prediction", meta = (ClampMin = "1"))
	int32 MaxDetailedCandidates = 32;

	UPROPERTY(EditAnywhere, Category = "Settings|Prediction", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	double MinimumRouteSeparationFraction = 0.02;
};
