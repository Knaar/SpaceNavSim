#include "UI/SpaceNavShipStatsWidget.h"

#include "Components/TextBlock.h"

void USpaceNavShipStatsWidget::InitCurrentMass(float Value)
{
	DisplayedCurrentMass = Value;
	TargetCurrentMass = Value;
	UpdateDisplayedText(CurrentMassText, DisplayedCurrentMass);
}

void USpaceNavShipStatsWidget::InitRemainingFuel(float Value)
{
	DisplayedRemainingFuel = Value;
	TargetRemainingFuel = Value;
	UpdateDisplayedText(RemainingFuelText, DisplayedRemainingFuel);
}

void USpaceNavShipStatsWidget::InitFuelConsumption(float Value)
{
	DisplayedFuelConsumption = Value;
	TargetFuelConsumption = Value;
	UpdateDisplayedText(FuelConsumptionText, DisplayedFuelConsumption);
}

void USpaceNavShipStatsWidget::UpdateCurrentMass(float Value)
{
	TargetCurrentMass = Value;
}

void USpaceNavShipStatsWidget::UpdateCurrentSpeed(float Value)
{
	TargetCurrentSpeed = Value;
}

void USpaceNavShipStatsWidget::UpdateRemainingFuel(float Value)
{
	TargetRemainingFuel = Value;
}

void USpaceNavShipStatsWidget::UpdateFuelConsumption(float Value)
{
	TargetFuelConsumption = Value;
}

void USpaceNavShipStatsWidget::NativeConstruct()
{
	Super::NativeConstruct();

	UpdateDisplayedText(CurrentMassText, DisplayedCurrentMass);
	UpdateDisplayedText(CurrentSpeedText, DisplayedCurrentSpeed);
	UpdateDisplayedText(RemainingFuelText, DisplayedRemainingFuel);
	UpdateDisplayedText(FuelConsumptionText, DisplayedFuelConsumption);
}

void USpaceNavShipStatsWidget::NativeTick(const FGeometry& MyGeometry, float InDeltaTime)
{
	Super::NativeTick(MyGeometry, InDeltaTime);

	InterpolateDisplayedValue(CurrentMassText, DisplayedCurrentMass, TargetCurrentMass, InDeltaTime);
	InterpolateDisplayedValue(CurrentSpeedText, DisplayedCurrentSpeed, TargetCurrentSpeed, InDeltaTime);
	InterpolateDisplayedValue(RemainingFuelText, DisplayedRemainingFuel, TargetRemainingFuel, InDeltaTime);
	InterpolateDisplayedValue(FuelConsumptionText, DisplayedFuelConsumption, TargetFuelConsumption, InDeltaTime);
}

void USpaceNavShipStatsWidget::InterpolateDisplayedValue(UTextBlock* TextBlock, float& DisplayedValue, float TargetValue, float DeltaTime)
{
	if (!TextBlock) return;

	const float interpolatedValue = FMath::FInterpTo(DisplayedValue, TargetValue, DeltaTime, InterpolationSpeed);
	if (interpolatedValue == DisplayedValue) return;

	DisplayedValue = interpolatedValue;
	UpdateDisplayedText(TextBlock, DisplayedValue);
}

void USpaceNavShipStatsWidget::UpdateDisplayedText(UTextBlock* TextBlock, float Value)
{
	if (!TextBlock) return;

	FNumberFormattingOptions numberFormattingOptions;
	numberFormattingOptions.MinimumFractionalDigits = 0;
	numberFormattingOptions.MaximumFractionalDigits = 1;

	const FText displayedText = FText::AsNumber(Value, &numberFormattingOptions);
	if (TextBlock->GetText().EqualTo(displayedText)) return;

	TextBlock->SetText(displayedText);
}
