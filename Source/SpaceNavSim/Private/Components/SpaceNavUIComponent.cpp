#include "Components/SpaceNavUIComponent.h"

#include "Engine/World.h"
#include "UI/SpaceNavMainWidget.h"

void USpaceNavUIComponent::InitializeMainWidget()
{
	UWorld* world = GetWorld();
	if (!world || !MainWidgetClass) return;

	MainWidget = CreateWidget<USpaceNavMainWidget>(world, MainWidgetClass);
	if (!MainWidget) return;

	MainWidget->AddToViewport();

	UpdateCurrentMass(0.0f);
	UpdateCurrentSpeed(0.0f);
	UpdateRemainingFuel(0.0f);
	UpdateFuelConsumption(0.0f);
}

void USpaceNavUIComponent::UpdateCurrentMass(float Value)
{
	if (!MainWidget) return;

	MainWidget->UpdateCurrentMass(Value);
}

void USpaceNavUIComponent::UpdateCurrentSpeed(float Value)
{
	if (!MainWidget) return;

	MainWidget->UpdateCurrentSpeed(Value);
}

void USpaceNavUIComponent::UpdateRemainingFuel(float Value)
{
	if (!MainWidget) return;

	MainWidget->UpdateRemainingFuel(Value);
}

void USpaceNavUIComponent::UpdateFuelConsumption(float Value)
{
	if (!MainWidget) return;

	MainWidget->UpdateFuelConsumption(Value);
}

void USpaceNavUIComponent::PressFuelEfficient()
{
	if (!MainWidget) return;

	MainWidget->OnPressedFuelEfficient();
}

void USpaceNavUIComponent::PressFast()
{
	if (!MainWidget) return;

	MainWidget->OnPressedFast();
}

void USpaceNavUIComponent::PressBalanced()
{
	if (!MainWidget) return;

	MainWidget->OnPressedBalanced();
}

void USpaceNavUIComponent::PressEngines()
{
	if (!MainWidget) return;

	MainWidget->OnPressedEngines();
}
