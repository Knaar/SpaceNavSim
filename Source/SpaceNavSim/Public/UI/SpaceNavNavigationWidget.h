#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "Styling/SlateTypes.h"
#include "TimerManager.h"
#include "SpaceNavNavigationWidget.generated.h"

class UButton;
class UTextBlock;

UCLASS(Abstract, Blueprintable)
class SPACENAVSIM_API USpaceNavNavigationWidget : public UUserWidget
{
	GENERATED_BODY()

public:
	UFUNCTION(BlueprintCallable, Category = "SpaceNavNavigationWidget|Feedback")
	void OnPressedFuelEfficient();

	UFUNCTION(BlueprintCallable, Category = "SpaceNavNavigationWidget|Feedback")
	void OnPressedFast();

	UFUNCTION(BlueprintCallable, Category = "SpaceNavNavigationWidget|Feedback")
	void OnPressedBalanced();

	UFUNCTION(BlueprintCallable, Category = "SpaceNavNavigationWidget|Feedback")
	void OnPressedEngines();

protected:
	virtual void NativePreConstruct() override;
	virtual void NativeDestruct() override;

	UPROPERTY(BlueprintReadOnly, Category = "SpaceNavNavigationWidget|Components", meta = (BindWidget))
	TObjectPtr<UButton> FuelEfficientButton;

	UPROPERTY(BlueprintReadOnly, Category = "SpaceNavNavigationWidget|Components", meta = (BindWidget))
	TObjectPtr<UButton> FastButton;

	UPROPERTY(BlueprintReadOnly, Category = "SpaceNavNavigationWidget|Components", meta = (BindWidget))
	TObjectPtr<UButton> BalancedButton;

	UPROPERTY(BlueprintReadOnly, Category = "SpaceNavNavigationWidget|Components", meta = (BindWidget))
	TObjectPtr<UButton> EnginesButton;

	UPROPERTY(BlueprintReadOnly, Category = "SpaceNavNavigationWidget|Components", meta = (BindWidget))
	TObjectPtr<UTextBlock> FuelEfficientText;

	UPROPERTY(BlueprintReadOnly, Category = "SpaceNavNavigationWidget|Components", meta = (BindWidget))
	TObjectPtr<UTextBlock> FastText;

	UPROPERTY(BlueprintReadOnly, Category = "SpaceNavNavigationWidget|Components", meta = (BindWidget))
	TObjectPtr<UTextBlock> BalancedText;

	UPROPERTY(BlueprintReadOnly, Category = "SpaceNavNavigationWidget|Components", meta = (BindWidget))
	TObjectPtr<UTextBlock> EnginesText;

private:
	struct FButtonPressState
	{
		FButtonStyle OriginalStyle;
		FTimerHandle ReleaseTimer;
	};

	void PressButton(UButton* Button, FButtonPressState& PressState);
	void ReleaseButton(UButton* Button, FButtonPressState& PressState);
	void ConfigureButton(UButton* Button, UTextBlock* Text, const FText& Label);

	FButtonPressState FuelEfficientPressState;
	FButtonPressState FastPressState;
	FButtonPressState BalancedPressState;
	FButtonPressState EnginesPressState;
};
