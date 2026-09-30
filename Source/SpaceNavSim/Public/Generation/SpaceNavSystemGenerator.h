#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SpaceNavSystemGenerator.generated.h"

class USpaceNavSystemGeneratorDataAsset;

UCLASS(PrioritizeCategories = "Settings|Data")
class SPACENAVSIM_API ASpaceNavSystemGenerator : public AActor
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, Category = "Settings|Data")
	TObjectPtr<USpaceNavSystemGeneratorDataAsset> GeneratorData;
};
