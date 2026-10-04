#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "SpaceNavPawnSettingsDataAsset.generated.h"

UCLASS(PrioritizeCategories = "Settings|Resources Settings|Navigation Settings|Movement")
class SPACENAVSIM_API USpaceNavPawnSettingsDataAsset : public UDataAsset
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, Category = "Settings|Resources", meta = (DisplayName = "Initial Fuel (kg)"))
	float InitialFuelKg = 0.0f;

	UPROPERTY(EditAnywhere, Category = "Settings|Resources", meta = (DisplayName = "Max Fuel (kg)"))
	float MaxFuelKg = 1000.0f;

	UPROPERTY(EditAnywhere, Category = "Settings|Navigation",
		meta = (DisplayName = "Planet Safety Buffer (radius fraction)"))
	double PlanetSafetyBufferFraction = 0.6;

	UPROPERTY(EditAnywhere, Category = "Settings|Movement")
	float MaxSpeed = 10000.0f;
};
