#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SpaceNavPlanet.generated.h"

class UMaterialInterface;
class UStaticMeshComponent;

UCLASS(Blueprintable)
class SPACENAVSIM_API ASpaceNavPlanet : public AActor
{
	GENERATED_BODY()

public:
	ASpaceNavPlanet();

	void InitializePlanet(double InMassEarth, double InRadiusEarth, double InOrbitalRadiusAU,
		double InInitialOrbitalPhaseDegrees, UMaterialInterface* InMaterial);

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SpaceNavPlanet|Components")
	TObjectPtr<UStaticMeshComponent> BodyMesh;

	UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly, Category = "SpaceNavPlanet|Runtime")
	double MassEarth = 0.0;

	UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly, Category = "SpaceNavPlanet|Runtime")
	double RadiusEarth = 0.0;

	UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly, Category = "SpaceNavPlanet|Runtime")
	double OrbitalRadiusAU = 0.0;

	UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly, Category = "SpaceNavPlanet|Runtime")
	double InitialOrbitalPhaseDegrees = 0.0;
};
