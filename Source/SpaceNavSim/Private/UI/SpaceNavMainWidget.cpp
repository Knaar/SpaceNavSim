#include "UI/SpaceNavMainWidget.h"

#include "UI/SpaceNavNavigationWidget.h"
#include "UI/SpaceNavShipStatsWidget.h"

void USpaceNavMainWidget::InitCurrentMass(float Value)
{
	if (!ShipStatsWidget) return;

	ShipStatsWidget->InitCurrentMass(Value);
}

void USpaceNavMainWidget::InitRemainingFuel(float Value)
{
	if (!ShipStatsWidget) return;

	ShipStatsWidget->InitRemainingFuel(Value);
}

void USpaceNavMainWidget::InitFuelConsumption(float Value)
{
	if (!ShipStatsWidget) return;

	ShipStatsWidget->InitFuelConsumption(Value);
}

void USpaceNavMainWidget::UpdateCurrentMass(float Value)
{
	if (!ShipStatsWidget) return;

	ShipStatsWidget->UpdateCurrentMass(Value);
}

void USpaceNavMainWidget::UpdateCurrentSpeed(float Value)
{
	if (!ShipStatsWidget) return;

	ShipStatsWidget->UpdateCurrentSpeed(Value);
}

void USpaceNavMainWidget::UpdateRemainingFuel(float Value)
{
	if (!ShipStatsWidget) return;

	ShipStatsWidget->UpdateRemainingFuel(Value);
}

void USpaceNavMainWidget::UpdateFuelConsumption(float Value)
{
	if (!ShipStatsWidget) return;

	ShipStatsWidget->UpdateFuelConsumption(Value);
}

void USpaceNavMainWidget::OnPressedFuelEfficient()
{
	if (!NavigationWidget) return;

	NavigationWidget->OnPressedFuelEfficient();
}

void USpaceNavMainWidget::OnPressedFast()
{
	if (!NavigationWidget) return;

	NavigationWidget->OnPressedFast();
}

void USpaceNavMainWidget::OnPressedBalanced()
{
	if (!NavigationWidget) return;

	NavigationWidget->OnPressedBalanced();
}

void USpaceNavMainWidget::OnPressedEngines()
{
	if (!NavigationWidget) return;

	NavigationWidget->OnPressedEngines();
}
