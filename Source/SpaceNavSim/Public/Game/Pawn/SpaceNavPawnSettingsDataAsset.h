#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "Math/Interval.h"
#include "Templates/SubclassOf.h"
#include "SpaceNavPawnSettingsDataAsset.generated.h"

class AActor;

UCLASS(PrioritizeCategories = "Settings|Mass Settings|Resources Settings|Navigation Settings|Movement Settings|Spawn")
class SPACENAVSIM_API USpaceNavPawnSettingsDataAsset : public UDataAsset
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, Category = "Settings|Mass", meta = (DisplayName = "Dry Mass (kg)"))
	float DryMassKg = 2000.0f;

	UPROPERTY(EditAnywhere, Category = "Settings|Resources", meta = (DisplayName = "Initial Fuel (kg)"))
	float InitialFuelKg = 0.0f;

	UPROPERTY(EditAnywhere, Category = "Settings|Resources", meta = (DisplayName = "Max Fuel (kg)"))
	float MaxFuelKg = 1000.0f;

	UPROPERTY(EditAnywhere, Category = "Settings|Navigation",
		meta = (DisplayName = "Planet Safety Buffer (radius fraction)"))
	double PlanetSafetyBufferFraction = 0.6;

	UPROPERTY(EditAnywhere, Category = "Settings|Movement", meta = (DisplayName = "Max Thrust"))
	float MaxThrust = 2000.0f;

	UPROPERTY(EditAnywhere, Category = "Settings|Movement")
	float MaxSpeed = 10000.0f;

	UPROPERTY(EditAnywhere, Category = "Settings|Spawn", meta = (DisplayName = "Starting Body"))
	TSubclassOf<AActor> StartingBodyActorClass;

	UPROPERTY(EditAnywhere, Category = "Settings|Spawn", meta = (DisplayName = "Starting Orbit Radius"))
	FFloatInterval StartingOrbitRadius = FFloatInterval(20.0f, 100.0f);

	UPROPERTY(EditAnywhere, Category = "Settings|Spawn", meta = (DisplayName = "Initial Spacecraft Velocity"))
	float InitialSpacecraftVelocity = 0.0f;
};
