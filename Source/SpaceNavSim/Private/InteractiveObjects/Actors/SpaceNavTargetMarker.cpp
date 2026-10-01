#include "InteractiveObjects/Actors/SpaceNavTargetMarker.h"

#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"

ASpaceNavTargetMarker::ASpaceNavTargetMarker()
{
    SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("SceneRoot"));
    SetRootComponent(SceneRoot);

    MarkerMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("MarkerMesh"));
    MarkerMesh->SetupAttachment(SceneRoot);
}
