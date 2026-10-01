#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SpaceNavStartMarker.generated.h"

class USceneComponent;
class UStaticMeshComponent;

UCLASS(Blueprintable)
class SPACENAVSIM_API ASpaceNavStartMarker : public AActor
{
	GENERATED_BODY()

public:
	ASpaceNavStartMarker();

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SpaceNavStartMarker|Components")
    TObjectPtr<USceneComponent> SceneRoot;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SpaceNavStartMarker|Components")
    TObjectPtr<UStaticMeshComponent> MarkerMesh;
};
