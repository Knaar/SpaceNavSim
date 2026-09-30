#include "InteractiveObjects/Actors/SpaceNavCentralBody.h"

#include "Components/StaticMeshComponent.h"

ASpaceNavCentralBody::ASpaceNavCentralBody()
{
	BodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BodyMesh"));
	SetRootComponent(BodyMesh);
}

void ASpaceNavCentralBody::InitializeBody(double InMassEarth, double InRadiusEarth)
{
	MassEarth = InMassEarth;
	RadiusEarth = InRadiusEarth;
}
