#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "SpaceNavShipStatsWidget.generated.h"

class UTextBlock;

UCLASS(Abstract, Blueprintable)
class SPACENAVSIM_API USpaceNavShipStatsWidget : public UUserWidget
{
	GENERATED_BODY()

public:
	void InitCurrentMass(float Value);
	void InitRemainingFuel(float Value);
	void InitFuelConsumption(float Value);

	UFUNCTION(BlueprintCallable, Category = "SpaceNavShipStatsWidget|Updates")
	void UpdateCurrentMass(float Value);

	UFUNCTION(BlueprintCallable, Category = "SpaceNavShipStatsWidget|Updates")
	void UpdateCurrentSpeed(float Value);

	UFUNCTION(BlueprintCallable, Category = "SpaceNavShipStatsWidget|Updates")
	void UpdateRemainingFuel(float Value);

	UFUNCTION(BlueprintCallable, Category = "SpaceNavShipStatsWidget|Updates")
	void UpdateFuelConsumption(float Value);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings|Display")
	float InterpolationSpeed = 6.0f;

protected:
	virtual void NativeConstruct() override;
	virtual void NativeTick(const FGeometry& MyGeometry, float InDeltaTime) override;

	UPROPERTY(BlueprintReadOnly, Category = "SpaceNavShipStatsWidget|Components", meta = (BindWidget))
	TObjectPtr<UTextBlock> CurrentMassText;

	UPROPERTY(BlueprintReadOnly, Category = "SpaceNavShipStatsWidget|Components", meta = (BindWidget))
	TObjectPtr<UTextBlock> CurrentSpeedText;

	UPROPERTY(BlueprintReadOnly, Category = "SpaceNavShipStatsWidget|Components", meta = (BindWidget))
	TObjectPtr<UTextBlock> RemainingFuelText;

	UPROPERTY(BlueprintReadOnly, Category = "SpaceNavShipStatsWidget|Components", meta = (BindWidget))
	TObjectPtr<UTextBlock> FuelConsumptionText;

private:
	void InterpolateDisplayedValue(UTextBlock* TextBlock, float& DisplayedValue, float TargetValue, float DeltaTime);
	void UpdateDisplayedText(UTextBlock* TextBlock, float Value);

	float DisplayedCurrentMass = 0.0f;
	float TargetCurrentMass = 0.0f;

	float DisplayedCurrentSpeed = 0.0f;
	float TargetCurrentSpeed = 0.0f;

	float DisplayedRemainingFuel = 0.0f;
	float TargetRemainingFuel = 0.0f;

	float DisplayedFuelConsumption = 0.0f;
	float TargetFuelConsumption = 0.0f;
};
