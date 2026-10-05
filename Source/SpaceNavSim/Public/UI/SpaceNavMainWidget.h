#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "SpaceNavMainWidget.generated.h"

class USpaceNavNavigationWidget;
class USpaceNavShipStatsWidget;

UCLASS(Abstract, Blueprintable)
class SPACENAVSIM_API USpaceNavMainWidget : public UUserWidget
{
	GENERATED_BODY()

public:
	void InitCurrentMass(float Value);
	void InitRemainingFuel(float Value);
	void InitFuelConsumption(float Value);

	UFUNCTION(BlueprintCallable, Category = "SpaceNavMainWidget|Updates")
	void UpdateCurrentMass(float Value);

	UFUNCTION(BlueprintCallable, Category = "SpaceNavMainWidget|Updates")
	void UpdateCurrentSpeed(float Value);

	UFUNCTION(BlueprintCallable, Category = "SpaceNavMainWidget|Updates")
	void UpdateRemainingFuel(float Value);

	UFUNCTION(BlueprintCallable, Category = "SpaceNavMainWidget|Updates")
	void UpdateFuelConsumption(float Value);

	UFUNCTION(BlueprintCallable, Category = "SpaceNavMainWidget|Feedback")
	void OnPressedFuelEfficient();

	UFUNCTION(BlueprintCallable, Category = "SpaceNavMainWidget|Feedback")
	void OnPressedFast();

	UFUNCTION(BlueprintCallable, Category = "SpaceNavMainWidget|Feedback")
	void OnPressedBalanced();

	UFUNCTION(BlueprintCallable, Category = "SpaceNavMainWidget|Feedback")
	void OnPressedEngines();

protected:
	UPROPERTY(BlueprintReadOnly, Category = "SpaceNavMainWidget|Components", meta = (BindWidget))
	TObjectPtr<USpaceNavNavigationWidget> NavigationWidget;

	UPROPERTY(BlueprintReadOnly, Category = "SpaceNavMainWidget|Components", meta = (BindWidget))
	TObjectPtr<USpaceNavShipStatsWidget> ShipStatsWidget;
};
