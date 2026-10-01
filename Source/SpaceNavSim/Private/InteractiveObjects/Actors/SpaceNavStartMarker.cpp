#include "InteractiveObjects/Actors/SpaceNavStartMarker.h"

#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"

ASpaceNavStartMarker::ASpaceNavStartMarker()
{
    SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("SceneRoot"));
    SetRootComponent(SceneRoot);

    MarkerMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("MarkerMesh"));
    MarkerMesh->SetupAttachment(SceneRoot);
}
