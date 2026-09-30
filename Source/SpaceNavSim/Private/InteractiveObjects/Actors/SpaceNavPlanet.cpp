#include "InteractiveObjects/Actors/SpaceNavPlanet.h"

#include "Components/StaticMeshComponent.h"

ASpaceNavPlanet::ASpaceNavPlanet()
{
	BodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BodyMesh"));
	SetRootComponent(BodyMesh);
}

void ASpaceNavPlanet::InitializePlanet(double InMassEarth, double InRadiusEarth, double InOrbitalRadiusAU,
	double InInitialOrbitalPhaseDegrees, UMaterialInterface* InMaterial)
{
	MassEarth = InMassEarth;
	RadiusEarth = InRadiusEarth;
	OrbitalRadiusAU = InOrbitalRadiusAU;
	InitialOrbitalPhaseDegrees = InInitialOrbitalPhaseDegrees;

	if (InMaterial != nullptr)
	{
		BodyMesh->SetMaterial(0, InMaterial);
	}
}
