#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SpaceNavCentralBody.generated.h"

class UStaticMeshComponent;

UCLASS(Blueprintable)
class SPACENAVSIM_API ASpaceNavCentralBody : public AActor
{
	GENERATED_BODY()

public:
	ASpaceNavCentralBody();

	void InitializeBody(double InMassEarth, double InRadiusEarth);

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SpaceNavCentralBody|Components")
	TObjectPtr<UStaticMeshComponent> BodyMesh;

	UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly, Category = "SpaceNavCentralBody|Runtime")
	double MassEarth = 0.0;

	UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly, Category = "SpaceNavCentralBody|Runtime")
	double RadiusEarth = 0.0;
};
