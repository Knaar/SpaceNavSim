#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "SpaceNavGameMode.generated.h"

class ASpaceNavSystemGenerator;
class APlayerController;
class APawn;
class USpaceNavPawnSettingsDataAsset;

UCLASS(PrioritizeCategories = "Settings|Data")
class SPACENAVSIM_API ASpaceNavGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, Category = "Settings|Data")
	TObjectPtr<USpaceNavPawnSettingsDataAsset> PawnSettings;

protected:
	virtual void BeginPlay() override;
	virtual void HandleStartingNewPlayer_Implementation(APlayerController* NewPlayer) override;

private:
	void BindStartZone();
	void OnStartZoneReady();
	void TrySpawnPlayer(APlayerController* PlayerController);
	FTransform GetSpacecraftSpawnTransform(const AActor* StartingBodyActor) const;
	void ApplyInitialSpacecraftVelocity(APawn* Spacecraft) const;
	AActor* FindStartingBodyActor() const;
	ASpaceNavSystemGenerator* FindGenerator() const;

	bool bLogPlayerSpawn = true;
};
