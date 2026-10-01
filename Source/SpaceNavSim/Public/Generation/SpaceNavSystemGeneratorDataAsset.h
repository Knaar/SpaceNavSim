#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "Templates/SubclassOf.h"
#include "SpaceNavSystemGeneratorDataAsset.generated.h"

class ASpaceNavCentralBody;
class ASpaceNavPlanet;
class ASpaceNavStartMarker;
class ASpaceNavTargetMarker;
class UMaterialInterface;

UCLASS()
class SPACENAVSIM_API USpaceNavSystemGeneratorDataAsset : public UDataAsset
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, Category = "Settings|Generation")
	int32 RandomSeed = 12345;

	UPROPERTY(EditAnywhere, Category = "Settings|Generation")
	double SystemBoundaryKm = 120.0;

	UPROPERTY(EditAnywhere, Category = "Settings|Generation", meta = (DisplayName = "Gravity Coefficient (m^3 / (t s^2))"))
	double GravityCoefficient = 10.0;

	UPROPERTY(EditAnywhere, Category = "Settings|Central Body", meta = (DisplayName = "Mass (Tonnes)"))
	double CentralBodyMassTonnes = 1000000.0;

	UPROPERTY(EditAnywhere, Category = "Settings|Central Body", meta = (DisplayName = "Radius (Meters)"))
	double CentralBodyRadiusMeters = 800.0;

	UPROPERTY(EditAnywhere, Category = "Settings|Central Body")
	TSubclassOf<ASpaceNavCentralBody> CentralBodyClass;

	UPROPERTY(EditAnywhere, Category = "Settings|Planets")
	int32 NumberOfPlanets = 5;

	UPROPERTY(EditAnywhere, Category = "Settings|Planets")
	TSubclassOf<ASpaceNavPlanet> PlanetClass;

	UPROPERTY(EditAnywhere, Category = "Settings|Planets")
	TArray<TObjectPtr<UMaterialInterface>> PlanetMaterials;

	UPROPERTY(EditAnywhere, Category = "Settings|Planets", meta = (DisplayName = "Min Mass (Tonnes)"))
	double MinPlanetMassTonnes = 100.0;

	UPROPERTY(EditAnywhere, Category = "Settings|Planets", meta = (DisplayName = "Max Mass (Tonnes)"))
	double MaxPlanetMassTonnes = 1000.0;

	UPROPERTY(EditAnywhere, Category = "Settings|Planets", meta = (DisplayName = "Min Radius (Meters)"))
	double MinPlanetRadiusMeters = 100.0;

	UPROPERTY(EditAnywhere, Category = "Settings|Planets", meta = (DisplayName = "Max Radius (Meters)"))
	double MaxPlanetRadiusMeters = 300.0;

	UPROPERTY(EditAnywhere, Category = "Settings|Planets")
	double MinOrbitalRadiusKm = 5.0;

	UPROPERTY(EditAnywhere, Category = "Settings|Planets")
	double MaxOrbitalRadiusKm = 100.0;

	UPROPERTY(EditAnywhere, Category = "Settings|Planets")
	double MinInitialOrbitalPhaseDegrees = 0.0;

	UPROPERTY(EditAnywhere, Category = "Settings|Planets")
	double MaxInitialOrbitalPhaseDegrees = 360.0;

	UPROPERTY(EditAnywhere, Category = "Settings|Markers")
	TSubclassOf<ASpaceNavStartMarker> StartMarkerClass;

	UPROPERTY(EditAnywhere, Category = "Settings|Markers")
	TSubclassOf<ASpaceNavTargetMarker> TargetMarkerClass;
};
