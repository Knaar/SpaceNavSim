#include "Game/GameMode/SpaceNavGameMode.h"

#include "Generation/SpaceNavSystemGenerator.h"
#include "InteractiveObjects/Actors/SpaceNavStartMarker.h"

#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerController.h"

DEFINE_LOG_CATEGORY_STATIC(LogSpaceNavGameMode, Log, All);

void ASpaceNavGameMode::BeginPlay()
{
	Super::BeginPlay();
	BindStartZone();
}

void ASpaceNavGameMode::HandleStartingNewPlayer_Implementation(APlayerController* NewPlayer)
{
	TrySpawnPlayer(NewPlayer);
}

void ASpaceNavGameMode::BindStartZone()
{
	if (bLogPlayerSpawn) UE_LOG(LogSpaceNavGameMode, Display, TEXT("Started"));
	ASpaceNavSystemGenerator* generator = FindGenerator();
	if (generator == nullptr)
	{
		UE_LOG(LogSpaceNavGameMode, Error, TEXT("Failed: generator"));
		return;
	}

	generator->OnStartZoneReady.AddUObject(this, &ASpaceNavGameMode::OnStartZoneReady);
	TrySpawnPlayer(GetWorld()->GetFirstPlayerController());
}

void ASpaceNavGameMode::OnStartZoneReady()
{
	TrySpawnPlayer(GetWorld()->GetFirstPlayerController());
}

void ASpaceNavGameMode::TrySpawnPlayer(APlayerController* PlayerController)
{
	if (PlayerController == nullptr || PlayerController->GetPawn() != nullptr) return;
	ASpaceNavSystemGenerator* generator = FindGenerator();
	if (generator == nullptr) return;
	ASpaceNavStartMarker* startMarker = generator->GetStartMarker();
	if (!IsValid(startMarker)) return;

	RestartPlayerAtPlayerStart(PlayerController, startMarker);
	if (PlayerController->GetPawn() == nullptr)
	{
		UE_LOG(LogSpaceNavGameMode, Error, TEXT("Failed: pawn"));
		return;
	}
	if (bLogPlayerSpawn) UE_LOG(LogSpaceNavGameMode, Display, TEXT("Pawn spawned"));
}

ASpaceNavSystemGenerator* ASpaceNavGameMode::FindGenerator() const
{
	for (TActorIterator<ASpaceNavSystemGenerator> generatorIterator(GetWorld()); generatorIterator; ++generatorIterator)
	{
		return *generatorIterator;
	}
	return nullptr;
}
