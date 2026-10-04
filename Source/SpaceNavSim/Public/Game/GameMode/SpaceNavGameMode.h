#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "SpaceNavGameMode.generated.h"

class ASpaceNavSystemGenerator;
class APlayerController;

UCLASS()
class SPACENAVSIM_API ASpaceNavGameMode : public AGameModeBase
{
	GENERATED_BODY()

protected:
	virtual void BeginPlay() override;
	virtual void HandleStartingNewPlayer_Implementation(APlayerController* NewPlayer) override;

private:
	void BindStartZone();
	void OnStartZoneReady();
	void TrySpawnPlayer(APlayerController* PlayerController);
	ASpaceNavSystemGenerator* FindGenerator() const;

	bool bLogPlayerSpawn = true;
};
