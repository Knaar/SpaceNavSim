#include "Components/SpaceNavOnboardComputerComponent.h"

#include "GameFramework/Actor.h"

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

void USpaceNavOnboardComputerComponent::UpdateDistance()
{
	const FVector currentLocation = GetOwner()->GetActorLocation();
	const double distanceCm = FVector::Dist(PreviousLocation, currentLocation);
	AccumulatedDistanceCm += distanceCm;
	TotalDistanceCm += distanceCm;
	PreviousLocation = currentLocation;
}
