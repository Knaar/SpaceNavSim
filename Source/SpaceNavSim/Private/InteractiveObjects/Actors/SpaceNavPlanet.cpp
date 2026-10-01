#include "InteractiveObjects/Actors/SpaceNavPlanet.h"

#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"

ASpaceNavPlanet::ASpaceNavPlanet()
{
	BodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BodyMesh"));
	SetRootComponent(BodyMesh);
}

void ASpaceNavPlanet::InitializePlanet(double InMassTonnes, double InRadiusMeters, double InOrbitalRadiusKm,
	double InInitialOrbitalPhaseDegrees, const FVector& InInitialOrbitalVelocityMetersPerSecond,
	UMaterialInterface* InMaterial)
{
	MassTonnes = InMassTonnes;
	RadiusMeters = InRadiusMeters;
	OrbitalRadiusKm = InOrbitalRadiusKm;
	InitialOrbitalPhaseDegrees = InInitialOrbitalPhaseDegrees;
	InitialOrbitalVelocityMetersPerSecond = InInitialOrbitalVelocityMetersPerSecond;
	SelectedMaterial = InMaterial;
	ApplyPlanetMaterial();
}

void ASpaceNavPlanet::PostInitializeComponents()
{
	Super::PostInitializeComponents();
	ApplyVisualRadius();
	ApplyPlanetMaterial();
}

void ASpaceNavPlanet::ApplyVisualRadius()
{
	if (BodyMesh == nullptr || !FMath::IsFinite(RadiusMeters) || RadiusMeters <= 0.0) return;
	const UStaticMesh* staticMesh = BodyMesh->GetStaticMesh();
	if (staticMesh == nullptr) return;
	const double meshRadiusUU = staticMesh->GetBounds().SphereRadius;
	if (meshRadiusUU <= 0.0) return;

	constexpr double UnrealUnitsPerMeter = 100.0;
	BodyMesh->SetWorldScale3D(FVector(RadiusMeters * UnrealUnitsPerMeter / meshRadiusUU));
}

void ASpaceNavPlanet::ApplyPlanetMaterial()
{
	if (SelectedMaterial != nullptr)
	{
		BodyMesh->SetMaterial(0, SelectedMaterial.Get());
	}
}
