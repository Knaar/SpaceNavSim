#include "Game/GameMode/SpaceNavGameMode.h"

#include "Components/SpaceNavEngineComponent.h"
#include "Game/Pawn/SpaceNavPawnSettingsDataAsset.h"
#include "Generation/SpaceNavSystemGenerator.h"
#include "InteractiveObjects/Actors/SpaceNavStartMarker.h"

#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Pawn.h"
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

	AActor* startingBodyActor = FindStartingBodyActor();
	if (startingBodyActor == nullptr) return;

	RestartPlayerAtTransform(PlayerController, GetSpacecraftSpawnTransform(startingBodyActor));
	if (PlayerController->GetPawn() == nullptr)
	{
		UE_LOG(LogSpaceNavGameMode, Error, TEXT("Failed: pawn"));
		return;
	}
	ApplyInitialSpacecraftVelocity(PlayerController->GetPawn());
	if (bLogPlayerSpawn) UE_LOG(LogSpaceNavGameMode, Display, TEXT("Pawn spawned"));
}

FTransform ASpaceNavGameMode::GetSpacecraftSpawnTransform(const AActor* StartingBodyActor) const
{
	const float spawnHeight = FMath::FRandRange(PawnSettings->StartingOrbitRadius.Min,
		PawnSettings->StartingOrbitRadius.Max);
	const FVector spawnLocation = StartingBodyActor->GetActorLocation() + FVector::UpVector * spawnHeight;
	return FTransform(StartingBodyActor->GetActorRotation(), spawnLocation);
}

void ASpaceNavGameMode::ApplyInitialSpacecraftVelocity(APawn* Spacecraft) const
{
	USpaceNavEngineComponent* engine = Spacecraft->FindComponentByClass<USpaceNavEngineComponent>();
	if (engine == nullptr) return;
	engine->SetInitialVelocity(Spacecraft->GetActorForwardVector() * PawnSettings->InitialSpacecraftVelocity);
}

AActor* ASpaceNavGameMode::FindStartingBodyActor() const
{
	if (PawnSettings == nullptr)
	{
		UE_LOG(LogSpaceNavGameMode, Warning, TEXT("Failed: pawn settings"));
		return nullptr;
	}
	if (PawnSettings->StartingBodyActorClass == nullptr)
	{
		UE_LOG(LogSpaceNavGameMode, Warning, TEXT("Failed: starting class"));
		return nullptr;
	}

	for (TActorIterator<AActor> actorIterator(GetWorld(), PawnSettings->StartingBodyActorClass); actorIterator; ++actorIterator)
	{
		return *actorIterator;
	}
	UE_LOG(LogSpaceNavGameMode, Warning, TEXT("Failed: starting actor"));
	return nullptr;
}

ASpaceNavSystemGenerator* ASpaceNavGameMode::FindGenerator() const
{
	for (TActorIterator<ASpaceNavSystemGenerator> generatorIterator(GetWorld()); generatorIterator; ++generatorIterator)
	{
		return *generatorIterator;
	}
	return nullptr;
}
