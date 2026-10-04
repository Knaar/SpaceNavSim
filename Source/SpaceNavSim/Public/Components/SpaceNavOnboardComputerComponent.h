#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "SpaceNavOnboardComputerComponent.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FSpaceNavSpeedUpdated, float, SpeedCmPerSecond);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FSpaceNavFuelConsumptionUpdated, float, KgPerKm);

UCLASS(BlueprintType, Blueprintable, ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class SPACENAVSIM_API USpaceNavOnboardComputerComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	USpaceNavOnboardComputerComponent();

	UFUNCTION(BlueprintCallable, Category = "SpaceNavOnboardComputer|Monitoring")
	void InitOnboardComputer(float SamplePeriodSeconds);

	UFUNCTION(BlueprintCallable, Category = "SpaceNavOnboardComputer|Fuel")
	void HandleFuelChanged(float RemainingFuelKg);

	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;

	UPROPERTY(BlueprintAssignable, Category = "SpaceNavOnboardComputer|Events")
	FSpaceNavSpeedUpdated OnSpeedUpdated;

	UPROPERTY(BlueprintAssignable, Category = "SpaceNavOnboardComputer|Events")
	FSpaceNavFuelConsumptionUpdated OnFuelConsumptionUpdated;

private:
	void UpdateDistance();

	FVector PreviousLocation = FVector::ZeroVector;
	double AccumulatedDistanceCm = 0.0;
	double TotalDistanceCm = 0.0;
	double ElapsedSeconds = 0.0;
	double MeasurementPeriodSeconds = 0.0;
	double FuelTrackingStartDistanceCm = 0.0;
	double TotalConsumedFuelKg = 0.0;
	float PreviousFuelKg = 0.0f;
	bool bHasFuelSample = false;
};
