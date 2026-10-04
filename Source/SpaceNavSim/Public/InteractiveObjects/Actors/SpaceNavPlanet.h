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

	struct FOrbitSnapshot
	{
		FVector Center = FVector::ZeroVector;
		FVector InitialOffset = FVector::ZeroVector;
		FVector Axis = FVector::UpVector;
		FVector CurrentLocation = FVector::ZeroVector;
		double AngularSpeedRadiansPerSecond = 0.0;
		double ElapsedSeconds = 0.0;

		FVector LocationAfter(double SecondsAhead) const;
	};

	void InitializePlanet(double InMassTonnes, double InRadiusMeters, double InOrbitalRadiusKm,
		double InInitialOrbitalPhaseDegrees, const FVector& InOrbitCenter, const FVector& InOrbitAxis,
		double InAngularSpeedRadiansPerSecond,
		const FVector& InInitialOrbitalVelocityMetersPerSecond,
		UMaterialInterface* InMaterial);

	FVector PredictLocationAfter(double SecondsAhead) const;
	FOrbitSnapshot GetOrbitSnapshot() const;

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
	virtual void Tick(float DeltaSeconds) override;

private:
	void ApplyVisualRadius();
	void ApplyPlanetMaterial();

	UPROPERTY(Transient)
	TObjectPtr<UMaterialInterface> SelectedMaterial;

	FVector OrbitCenter = FVector::ZeroVector;
	FVector InitialOrbitOffset = FVector::ZeroVector;
	FVector OrbitAxis = FVector::UpVector;
	double AngularSpeedRadiansPerSecond = 0.0;
	double OrbitElapsedSeconds = 0.0;
};
