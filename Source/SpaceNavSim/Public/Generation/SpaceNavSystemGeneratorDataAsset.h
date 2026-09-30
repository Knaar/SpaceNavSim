#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "Templates/SubclassOf.h"
#include "SpaceNavSystemGeneratorDataAsset.generated.h"

class ASpaceNavCentralBody;
class ASpaceNavPlanet;
class UMaterialInterface;

UCLASS()
class SPACENAVSIM_API USpaceNavSystemGeneratorDataAsset : public UDataAsset
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, Category = "Settings|Generation")
	int32 RandomSeed = 12345;

	UPROPERTY(EditAnywhere, Category = "Settings|Generation")
	double SystemBoundaryAU = 120.0;

	UPROPERTY(EditAnywhere, Category = "Settings|Central Body", meta = (DisplayName = "Mass (Earth Masses)"))
	double CentralBodyMass = 333000.0;

	UPROPERTY(EditAnywhere, Category = "Settings|Central Body", meta = (DisplayName = "Radius (Earth Radii)"))
	double CentralBodyRadius = 109.0;

	UPROPERTY(EditAnywhere, Category = "Settings|Central Body")
	TSubclassOf<ASpaceNavCentralBody> CentralBodyClass;

	UPROPERTY(EditAnywhere, Category = "Settings|Planets")
	int32 NumberOfPlanets = 5;

	UPROPERTY(EditAnywhere, Category = "Settings|Planets")
	TSubclassOf<ASpaceNavPlanet> PlanetClass;

	UPROPERTY(EditAnywhere, Category = "Settings|Planets")
	TArray<TObjectPtr<UMaterialInterface>> PlanetMaterials;

	UPROPERTY(EditAnywhere, Category = "Settings|Planets", meta = (DisplayName = "Min Mass (Earth Masses)"))
	double MinPlanetMass = 0.5;

	UPROPERTY(EditAnywhere, Category = "Settings|Planets", meta = (DisplayName = "Max Mass (Earth Masses)"))
	double MaxPlanetMass = 2.0;

	UPROPERTY(EditAnywhere, Category = "Settings|Planets", meta = (DisplayName = "Min Radius (Earth Radii)"))
	double MinPlanetRadius = 0.8;

	UPROPERTY(EditAnywhere, Category = "Settings|Planets", meta = (DisplayName = "Max Radius (Earth Radii)"))
	double MaxPlanetRadius = 1.3;

	UPROPERTY(EditAnywhere, Category = "Settings|Planets")
	double MinOrbitalRadiusAU = 0.4;

	UPROPERTY(EditAnywhere, Category = "Settings|Planets")
	double MaxOrbitalRadiusAU = 10.0;

	UPROPERTY(EditAnywhere, Category = "Settings|Planets")
	double MinInitialOrbitalPhaseDegrees = 0.0;

	UPROPERTY(EditAnywhere, Category = "Settings|Planets")
	double MaxInitialOrbitalPhaseDegrees = 360.0;
};
