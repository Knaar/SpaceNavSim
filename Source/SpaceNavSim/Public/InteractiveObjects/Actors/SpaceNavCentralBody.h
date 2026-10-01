#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SpaceNavCentralBody.generated.h"

class UStaticMeshComponent;
class UPointLightComponent;

UCLASS(Blueprintable)
class SPACENAVSIM_API ASpaceNavCentralBody : public AActor
{
	GENERATED_BODY()

public:
	ASpaceNavCentralBody();

	void InitializeBody(double InMassTonnes, double InRadiusMeters);

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SpaceNavCentralBody|Components")
	TObjectPtr<UStaticMeshComponent> BodyMesh;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SpaceNavCentralBody|Components")
	TObjectPtr<UPointLightComponent> StarLight;

	UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly, Category = "SpaceNavCentralBody|Runtime")
	double MassTonnes = 0.0;

	UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly, Category = "SpaceNavCentralBody|Runtime")
	double RadiusMeters = 0.0;

protected:
	virtual void PostInitializeComponents() override;

private:
	void ApplyVisualRadius();
};
