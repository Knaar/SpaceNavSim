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

USTRUCT()
struct SPACENAVSIM_API FSpaceNavObjectGenerationSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Settings|Objects", meta = (DisplayName = "Number of Class"))
	int32 NumberOfClass = 5;

	UPROPERTY(EditAnywhere, Category = "Settings|Objects")
	double PlanetSpacingMultiplier = 3.0;

	UPROPERTY(EditAnywhere, Category = "Settings|Objects")
	TSubclassOf<ASpaceNavPlanet> PlanetClass;

	UPROPERTY(EditAnywhere, Category = "Settings|Objects")
	TArray<TObjectPtr<UMaterialInterface>> PlanetMaterials;

	UPROPERTY(EditAnywhere, Category = "Settings|Objects", meta = (DisplayName = "Min Mass (Tonnes)"))
	double MinPlanetMassTonnes = 100.0;

	UPROPERTY(EditAnywhere, Category = "Settings|Objects", meta = (DisplayName = "Max Mass (Tonnes)"))
	double MaxPlanetMassTonnes = 1000.0;

	UPROPERTY(EditAnywhere, Category = "Settings|Objects", meta = (DisplayName = "Min Radius (Meters)"))
	double MinPlanetRadiusMeters = 100.0;

	UPROPERTY(EditAnywhere, Category = "Settings|Objects", meta = (DisplayName = "Max Radius (Meters)"))
	double MaxPlanetRadiusMeters = 300.0;

	UPROPERTY(EditAnywhere, Category = "Settings|Objects")
	double MinOrbitalRadiusKm = 5.0;

	UPROPERTY(EditAnywhere, Category = "Settings|Objects")
	double MaxOrbitalRadiusKm = 100.0;

	UPROPERTY(EditAnywhere, Category = "Settings|Objects", meta = (DisplayName = "Min Outer Orbit Speed (m/s)"))
	double MinPlanetOrbitalSpeedMetersPerSecond = 1.0;

	UPROPERTY(EditAnywhere, Category = "Settings|Objects", meta = (DisplayName = "Max Outer Orbit Speed (m/s)"))
	double MaxPlanetOrbitalSpeedMetersPerSecond = 5.0;

	UPROPERTY(EditAnywhere, Category = "Settings|Objects")
	double MinInitialOrbitalPhaseDegrees = 0.0;

	UPROPERTY(EditAnywhere, Category = "Settings|Objects")
	double MaxInitialOrbitalPhaseDegrees = 360.0;
};

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

	UPROPERTY(EditAnywhere, Category = "Settings|Objects")
	TArray<FSpaceNavObjectGenerationSettings> Objects;

	UPROPERTY(EditAnywhere, Category = "Settings|Markers")
	TSubclassOf<ASpaceNavStartMarker> StartMarkerClass;

	UPROPERTY(EditAnywhere, Category = "Settings|Markers")
	TSubclassOf<ASpaceNavTargetMarker> TargetMarkerClass;
};
