#include "UI/SpaceNavNavigationWidget.h"
#include "Components/Button.h"
#include "Components/TextBlock.h"
#include "Engine/World.h"

void USpaceNavNavigationWidget::OnPressedFuelEfficient()
{
	PressButton(FuelEfficientButton, FuelEfficientPressState);
}

void USpaceNavNavigationWidget::OnPressedFast()
{
	PressButton(FastButton, FastPressState);
}

void USpaceNavNavigationWidget::OnPressedBalanced()
{
	PressButton(BalancedButton, BalancedPressState);
}

void USpaceNavNavigationWidget::OnPressedEngines()
{
	PressButton(EnginesButton, EnginesPressState);
}

void USpaceNavNavigationWidget::NativePreConstruct()
{
	Super::NativePreConstruct();
	ConfigureButton(FuelEfficientButton, FuelEfficientText,
		NSLOCTEXT("SpaceNavNavigationWidget", "FuelEfficientAction", "Launch Fuel Efficient Trace\nPress 1"));
	ConfigureButton(FastButton, FastText,
		NSLOCTEXT("SpaceNavNavigationWidget", "FastAction", "Launch Fast Trace\nPress 2"));
	ConfigureButton(BalancedButton, BalancedText,
		NSLOCTEXT("SpaceNavNavigationWidget", "BalancedAction", "Launch Balanced Trace\nPress 3"));
	ConfigureButton(EnginesButton, EnginesText,
		NSLOCTEXT("SpaceNavNavigationWidget", "EnginesAction", "Launch Engines\nPress 4"));
}

void USpaceNavNavigationWidget::NativeDestruct()
{
	ReleaseButton(FuelEfficientButton, FuelEfficientPressState);
	ReleaseButton(FastButton, FastPressState);
	ReleaseButton(BalancedButton, BalancedPressState);
	ReleaseButton(EnginesButton, EnginesPressState);
	Super::NativeDestruct();
}

void USpaceNavNavigationWidget::PressButton(UButton* Button, FButtonPressState& PressState)
{
	UWorld* world = GetWorld();
	if (Button == nullptr || world == nullptr) return;

	FTimerManager& timerManager = world->GetTimerManager();
	if (!timerManager.IsTimerActive(PressState.ReleaseTimer))
		PressState.OriginalStyle = Button->GetStyle();

	FButtonStyle pressedStyle = PressState.OriginalStyle;
	pressedStyle.Normal = pressedStyle.Pressed;
	pressedStyle.Hovered = pressedStyle.Pressed;
	pressedStyle.Disabled = pressedStyle.Pressed;
	pressedStyle.NormalPadding = pressedStyle.PressedPadding;
	Button->SetStyle(pressedStyle);

	FTimerDelegate releaseDelegate = FTimerDelegate::CreateWeakLambda(this,
		[this, Button, &PressState]()
		{
			ReleaseButton(Button, PressState);
		});
	timerManager.SetTimer(PressState.ReleaseTimer, releaseDelegate, 0.1f, false);
}

void USpaceNavNavigationWidget::ReleaseButton(UButton* Button, FButtonPressState& PressState)
{
	if (!PressState.ReleaseTimer.IsValid()) return;

	if (UWorld* world = GetWorld())
		world->GetTimerManager().ClearTimer(PressState.ReleaseTimer);
	PressState.ReleaseTimer.Invalidate();

	if (Button != nullptr) Button->SetStyle(PressState.OriginalStyle);
}

void USpaceNavNavigationWidget::ConfigureButton(UButton* Button, UTextBlock* Text, const FText& Label)
{
	if (Button != nullptr) Button->SetVisibility(ESlateVisibility::HitTestInvisible);
	if (Text != nullptr) Text->SetText(Label);
}
