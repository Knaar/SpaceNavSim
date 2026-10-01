#include "InteractiveObjects/Actors/SpaceNavCentralBody.h"

#include "Components/PointLightComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"

ASpaceNavCentralBody::ASpaceNavCentralBody()
{
	BodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BodyMesh"));
	SetRootComponent(BodyMesh);

	StarLight = CreateDefaultSubobject<UPointLightComponent>(TEXT("StarLight"));
	StarLight->SetupAttachment(BodyMesh);
	StarLight->SetAbsolute(false, false, true);
	StarLight->SetMobility(EComponentMobility::Movable);
}

void ASpaceNavCentralBody::InitializeBody(double InMassTonnes, double InRadiusMeters)
{
	MassTonnes = InMassTonnes;
	RadiusMeters = InRadiusMeters;
}

void ASpaceNavCentralBody::PostInitializeComponents()
{
	Super::PostInitializeComponents();
	ApplyVisualRadius();
}

void ASpaceNavCentralBody::ApplyVisualRadius()
{
	if (BodyMesh == nullptr || !FMath::IsFinite(RadiusMeters) || RadiusMeters <= 0.0) return;
	const UStaticMesh* staticMesh = BodyMesh->GetStaticMesh();
	if (staticMesh == nullptr) return;
	const double meshRadiusUU = staticMesh->GetBounds().SphereRadius;
	if (meshRadiusUU <= 0.0) return;

	constexpr double UnrealUnitsPerMeter = 100.0;
	BodyMesh->SetWorldScale3D(FVector(RadiusMeters * UnrealUnitsPerMeter / meshRadiusUU));
}
