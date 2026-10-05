#include "Components/SpaceNavResourceStoreComponent.h"
#include "Game/Pawn/SpaceNavPawnSettingsDataAsset.h"

#include <cmath>
#include <limits>

DEFINE_LOG_CATEGORY_STATIC(LogSpaceNavResourceStore, Log, All);

bool USpaceNavResourceStoreComponent::InitSettings()
{
	if (!PawnSettings)
	{
		UE_LOG(LogSpaceNavResourceStore, Error, TEXT("Failed: no settings"));
		return false;
	}

	const float initialFuelKg = PawnSettings->InitialFuelKg;
	const float maxFuelKg = PawnSettings->MaxFuelKg;
	if (!FMath::IsFinite(initialFuelKg) || !FMath::IsFinite(maxFuelKg) ||
		initialFuelKg < 0.0f || maxFuelKg < initialFuelKg)
	{
		UE_LOG(LogSpaceNavResourceStore, Error, TEXT("Failed: fuel data"));
		return false;
	}

	Resources.Fuel = initialFuelKg;
	Resources.MaxFuel = maxFuelKg;
	OnFuelAmountChanged.Broadcast(Resources.Fuel);
	if (bLogFuel) UE_LOG(LogSpaceNavResourceStore, Display, TEXT("Settings initialized"));
	return true;
}

float USpaceNavResourceStoreComponent::AddFuel(float AmountKg)
{
	const float previousFuelKg = Resources.Fuel;
	float addedFuelKg = 0.0f;
	if (AmountKg > 0.0f && previousFuelKg < Resources.MaxFuel)
	{
		Resources.Fuel = FMath::Min(previousFuelKg + AmountKg, Resources.MaxFuel);
		addedFuelKg = Resources.Fuel - previousFuelKg;
	}

	if (bLogFuel)
	{
		UE_LOG(LogSpaceNavResourceStore, Display,
			TEXT("AddFuel before=%.9g requested=%.9g added=%.9g after=%.9g"),
			previousFuelKg, AmountKg, addedFuelKg, Resources.Fuel);
	}
	if (addedFuelKg > 0.0f)
	{
		OnFuelAmountChanged.Broadcast(Resources.Fuel);
		BroadcastFuelChangedAtStep(previousFuelKg);
	}
	return addedFuelKg;
}

bool USpaceNavResourceStoreComponent::ConsumeFuel(float AmountKg)
{
	const float previousFuelKg = Resources.Fuel;
	float consumedFuelKg = 0.0f;
	bool bFuelDepleted = false;
	if (AmountKg > 0.0f && previousFuelKg > 0.0f)
	{
		Resources.Fuel = FMath::Max(0.0f, previousFuelKg - AmountKg);
		consumedFuelKg = previousFuelKg - Resources.Fuel;
		bFuelDepleted = Resources.Fuel == 0.0f;
	}

	if (consumedFuelKg > 0.0f)
	{
		OnFuelAmountChanged.Broadcast(Resources.Fuel);
		BroadcastFuelChangedAtStep(previousFuelKg);
	}
	if (bFuelDepleted)
	{
		if (bLogFuel) UE_LOG(LogSpaceNavResourceStore, Display, TEXT("Fuel empty"));
		OnFuelDepleted.Broadcast();
	}
	return consumedFuelKg > 0.0f;
}

float USpaceNavResourceStoreComponent::GetRemainingFuel() const
{
	return Resources.Fuel;
}

void USpaceNavResourceStoreComponent::BeginPlay()
{
	Super::BeginPlay();
	if (bLogFuel) UE_LOG(LogSpaceNavResourceStore, Display, TEXT("Fuel ready"));
}

void USpaceNavResourceStoreComponent::BroadcastFuelChangedAtStep(float PreviousFuelKg)
{
	const float currentFuelKg = Resources.Fuel;
	if (currentFuelKg == PreviousFuelKg || !(FuelChangeStepKg > 0.0f) || !FMath::IsFinite(FuelChangeStepKg)) return;

	const float fuelScaleKg = FMath::Max(1.0f, FMath::Max(FMath::Abs(PreviousFuelKg), FMath::Abs(currentFuelKg)));
	const float toleranceKg = FMath::Min(FuelChangeStepKg * 0.25f, 4.0f * std::numeric_limits<float>::epsilon() * fuelScaleKg);
	if (currentFuelKg > PreviousFuelKg)
	{
		const float previousStep = std::floor((PreviousFuelKg + toleranceKg) / FuelChangeStepKg);
		const float currentStep = std::floor((currentFuelKg + toleranceKg) / FuelChangeStepKg);
		if (currentStep > previousStep) OnFuelChanged.Broadcast(currentStep * FuelChangeStepKg);
		return;
	}

	const float previousStep = std::ceil((PreviousFuelKg - toleranceKg) / FuelChangeStepKg);
	const float currentStep = std::ceil((currentFuelKg - toleranceKg) / FuelChangeStepKg);
	if (currentStep < previousStep) OnFuelChanged.Broadcast(currentStep == 0.0f ? 0.0f : currentStep * FuelChangeStepKg);
}
