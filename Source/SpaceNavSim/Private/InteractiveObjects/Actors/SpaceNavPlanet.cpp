#include "InteractiveObjects/Actors/SpaceNavPlanet.h"

#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"

ASpaceNavPlanet::ASpaceNavPlanet()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;

	BodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BodyMesh"));
	SetRootComponent(BodyMesh);
}

void ASpaceNavPlanet::InitializePlanet(double InMassTonnes, double InRadiusMeters, double InOrbitalRadiusKm,
	double InInitialOrbitalPhaseDegrees, const FVector& InOrbitCenter, const FVector& InOrbitAxis,
	double InAngularSpeedRadiansPerSecond,
	const FVector& InInitialOrbitalVelocityMetersPerSecond,
	UMaterialInterface* InMaterial)
{
	MassTonnes = InMassTonnes;
	RadiusMeters = InRadiusMeters;
	OrbitalRadiusKm = InOrbitalRadiusKm;
	InitialOrbitalPhaseDegrees = InInitialOrbitalPhaseDegrees;
	InitialOrbitalVelocityMetersPerSecond = InInitialOrbitalVelocityMetersPerSecond;
	BodyMesh->SetMobility(EComponentMobility::Movable);
	OrbitCenter = InOrbitCenter;
	InitialOrbitOffset = GetActorLocation() - OrbitCenter;
	OrbitAxis = InOrbitAxis.GetSafeNormal();
	AngularSpeedRadiansPerSecond = InAngularSpeedRadiansPerSecond;
	OrbitElapsedSeconds = 0.0;
	SelectedMaterial = InMaterial;
	ApplyVisualRadius();
	ApplyPlanetMaterial();
}

void ASpaceNavPlanet::PostInitializeComponents()
{
	Super::PostInitializeComponents();
	ApplyVisualRadius();
	ApplyPlanetMaterial();
}

void ASpaceNavPlanet::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (AngularSpeedRadiansPerSecond == 0.0) return;

	OrbitElapsedSeconds += DeltaSeconds;
	SetActorLocation(PredictLocationAfter(0.0));
}

FVector ASpaceNavPlanet::PredictLocationAfter(double SecondsAhead) const
{
	if (AngularSpeedRadiansPerSecond == 0.0) return GetActorLocation();
	const FQuat orbitRotation(OrbitAxis,
		(OrbitElapsedSeconds + SecondsAhead) * AngularSpeedRadiansPerSecond);
	return OrbitCenter + orbitRotation.RotateVector(InitialOrbitOffset);
}

ASpaceNavPlanet::FOrbitSnapshot ASpaceNavPlanet::GetOrbitSnapshot() const
{
	FOrbitSnapshot snapshot;
	snapshot.Center = OrbitCenter;
	snapshot.InitialOffset = InitialOrbitOffset;
	snapshot.Axis = OrbitAxis;
	snapshot.CurrentLocation = GetActorLocation();
	snapshot.AngularSpeedRadiansPerSecond = AngularSpeedRadiansPerSecond;
	snapshot.ElapsedSeconds = OrbitElapsedSeconds;
	return snapshot;
}

FVector ASpaceNavPlanet::FOrbitSnapshot::LocationAfter(double SecondsAhead) const
{
	if (AngularSpeedRadiansPerSecond == 0.0) return CurrentLocation;
	const FQuat orbitRotation(Axis,
		(ElapsedSeconds + SecondsAhead) * AngularSpeedRadiansPerSecond);
	return Center + orbitRotation.RotateVector(InitialOffset);
}

void ASpaceNavPlanet::ApplyVisualRadius()
{
	if (BodyMesh == nullptr || !FMath::IsFinite(RadiusMeters) || RadiusMeters <= 0.0) return;
	const UStaticMesh* staticMesh = BodyMesh->GetStaticMesh();
	if (staticMesh == nullptr) return;
	const double meshRadiusUU = staticMesh->GetBounds().SphereRadius;
	if (meshRadiusUU <= 0.0) return;

	constexpr double planetUnitsPerMeter = 100.0;
	BodyMesh->SetWorldScale3D(FVector(RadiusMeters * planetUnitsPerMeter / meshRadiusUU));
}

void ASpaceNavPlanet::ApplyPlanetMaterial()
{
	if (SelectedMaterial != nullptr)
	{
		BodyMesh->SetMaterial(0, SelectedMaterial.Get());
	}
}
