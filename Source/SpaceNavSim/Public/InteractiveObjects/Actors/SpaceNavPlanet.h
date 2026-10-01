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

	void InitializePlanet(double InMassTonnes, double InRadiusMeters, double InOrbitalRadiusKm,
		double InInitialOrbitalPhaseDegrees, const FVector& InInitialOrbitalVelocityMetersPerSecond,
		UMaterialInterface* InMaterial);

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SpaceNavPlanet|Components")
	TObjectPtr<UStaticMeshComponent> BodyMesh;

	UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly, Category = "SpaceNavPlanet|Runtime")
	double MassTonnes = 0.0;

	UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly, Category = "SpaceNavPlanet|Runtime")
	double RadiusMeters = 0.0;

	UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly, Category = "SpaceNavPlanet|Runtime")
	double OrbitalRadiusKm = 0.0;

	UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly, Category = "SpaceNavPlanet|Runtime")
	double InitialOrbitalPhaseDegrees = 0.0;

	UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly, Category = "SpaceNavPlanet|Runtime")
	FVector InitialOrbitalVelocityMetersPerSecond = FVector::ZeroVector;

protected:
	virtual void PostInitializeComponents() override;

private:
	void ApplyVisualRadius();
	void ApplyPlanetMaterial();

	UPROPERTY(Transient)
	TObjectPtr<UMaterialInterface> SelectedMaterial;
};
