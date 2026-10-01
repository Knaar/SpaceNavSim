#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SpaceNavTargetMarker.generated.h"

class USceneComponent;
class UStaticMeshComponent;

UCLASS(Blueprintable)
class SPACENAVSIM_API ASpaceNavTargetMarker : public AActor
{
	GENERATED_BODY()

public:
	ASpaceNavTargetMarker();

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SpaceNavTargetMarker|Components")
    TObjectPtr<USceneComponent> SceneRoot;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SpaceNavTargetMarker|Components")
    TObjectPtr<UStaticMeshComponent> MarkerMesh;
};
