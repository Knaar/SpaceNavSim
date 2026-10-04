#include "Components/SpaceNavOnboardComputerComponent.h"

#include "Components/SpaceNavResourceStoreComponent.h"
#include "Game/Pawn/SpaceNavPawnSettingsDataAsset.h"
#include "GameFramework/Actor.h"

DEFINE_LOG_CATEGORY_STATIC(LogSpaceNavOnboardComputer, Log, All);

namespace
{
	constexpr double CentimetersPerKilometer = 100000.0;
}

USpaceNavOnboardComputerComponent::USpaceNavOnboardComputerComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = false;
}

void USpaceNavOnboardComputerComponent::InitOnboardComputer(float SamplePeriodSeconds)
{
	MeasurementPeriodSeconds = SamplePeriodSeconds;
	PreviousLocation = GetOwner()->GetActorLocation();
	AccumulatedDistanceCm = 0.0;
	TotalDistanceCm = 0.0;
	ElapsedSeconds = 0.0;
	FuelTrackingStartDistanceCm = 0.0;
	TotalConsumedFuelKg = 0.0;
	PreviousFuelKg = 0.0f;
	bHasFuelSample = false;
	SetComponentTickEnabled(true);
	InitializeMassTracking();
}

void USpaceNavOnboardComputerComponent::HandleFuelChanged(float RemainingFuelKg)
{
	UpdateDistance();
	if (!bHasFuelSample)
	{
		PreviousFuelKg = RemainingFuelKg;
		FuelTrackingStartDistanceCm = TotalDistanceCm;
		bHasFuelSample = true;
		return;
	}

	if (RemainingFuelKg < PreviousFuelKg)
	{
		TotalConsumedFuelKg += PreviousFuelKg - RemainingFuelKg;
	}
	PreviousFuelKg = RemainingFuelKg;

	const double distanceCm = TotalDistanceCm - FuelTrackingStartDistanceCm;
	if (distanceCm > 0.0)
	{
		OnFuelConsumptionUpdated.Broadcast(static_cast<float>(
			TotalConsumedFuelKg * CentimetersPerKilometer / distanceCm));
	}
}

void USpaceNavOnboardComputerComponent::TickComponent(float DeltaTime, ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	UpdateDistance();
	ElapsedSeconds += DeltaTime;
	if (ElapsedSeconds < MeasurementPeriodSeconds || ElapsedSeconds <= 0.0) return;

	OnSpeedUpdated.Broadcast(static_cast<float>(AccumulatedDistanceCm / ElapsedSeconds));
	AccumulatedDistanceCm = 0.0;
	ElapsedSeconds = 0.0;
}

void USpaceNavOnboardComputerComponent::InitializeMassTracking()
{
	if (ResourceStore.IsValid()) ResourceStore->OnFuelAmountChanged.RemoveAll(this);
	ResourceStore = GetOwner()->FindComponentByClass<USpaceNavResourceStoreComponent>();
	bHasSpacecraftMassSample = false;
	if (!CheckMassResources()) return;

	ResourceStore->OnFuelAmountChanged.AddUObject(this,
		&USpaceNavOnboardComputerComponent::HandleFuelAmountChanged);
	HandleFuelAmountChanged(ResourceStore->Resources.Fuel);
}

void USpaceNavOnboardComputerComponent::HandleFuelAmountChanged(float RemainingFuelKg)
{
	if (!CheckMassResources()) return;
	const double currentSpacecraftMassKg =
		static_cast<double>(ResourceStore->PawnSettings->DryMassKg) + RemainingFuelKg;
	if (bHasSpacecraftMassSample && currentSpacecraftMassKg == PreviousSpacecraftMassKg) return;

	PreviousSpacecraftMassKg = currentSpacecraftMassKg;
	bHasSpacecraftMassSample = true;
	OnSpacecraftMassChanged.Broadcast(currentSpacecraftMassKg);
}

bool USpaceNavOnboardComputerComponent::CheckMassResources() const
{
	if (!ResourceStore.IsValid())
	{
		UE_LOG(LogSpaceNavOnboardComputer, Warning, TEXT("Failed: mass resources"));
		return false;
	}
	if (ResourceStore->PawnSettings == nullptr)
	{
		UE_LOG(LogSpaceNavOnboardComputer, Warning, TEXT("Failed: mass settings"));
		return false;
	}
	return true;
}

void USpaceNavOnboardComputerComponent::UpdateDistance()
{
	const FVector currentLocation = GetOwner()->GetActorLocation();
	const double distanceCm = FVector::Dist(PreviousLocation, currentLocation);
	AccumulatedDistanceCm += distanceCm;
	TotalDistanceCm += distanceCm;
	PreviousLocation = currentLocation;
}
