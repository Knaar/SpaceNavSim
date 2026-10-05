#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "SpaceNavResourceStoreComponent.generated.h"

class USpaceNavPawnSettingsDataAsset;

USTRUCT(BlueprintType)
struct SPACENAVSIM_API FSpaceNavResources
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SpaceNavResourceStore|Resources")
	float Fuel = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SpaceNavResourceStore|Resources")
	float MaxFuel = 1000.0f;
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FSpaceNavFuelChanged, float, FuelKg);
DECLARE_MULTICAST_DELEGATE_OneParam(FSpaceNavFuelAmountChanged, float);
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FSpaceNavFuelDepleted);

UCLASS(Blueprintable, ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class SPACENAVSIM_API USpaceNavResourceStoreComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UFUNCTION(BlueprintCallable, Category = "SpaceNavResourceStore|Resources")
	bool InitSettings();

	UFUNCTION(BlueprintCallable, Category = "SpaceNavResourceStore|Resources")
	float AddFuel(float AmountKg);

	UFUNCTION(BlueprintCallable, Category = "SpaceNavResourceStore|Resources")
	bool ConsumeFuel(float AmountKg);

	float GetRemainingFuel() const;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SpaceNavResourceStore|Resources")
	FSpaceNavResources Resources;

	UPROPERTY(EditAnywhere, Category = "Settings|Data")
	TObjectPtr<USpaceNavPawnSettingsDataAsset> PawnSettings;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings|Fuel")
	float FuelChangeStepKg = 1.0f;

	UPROPERTY(BlueprintAssignable, Category = "SpaceNavResourceStore|Events")
	FSpaceNavFuelChanged OnFuelChanged;

	FSpaceNavFuelAmountChanged OnFuelAmountChanged;

	UPROPERTY(BlueprintAssignable, Category = "SpaceNavResourceStore|Events")
	FSpaceNavFuelDepleted OnFuelDepleted;

protected:
	virtual void BeginPlay() override;

private:
	void BroadcastFuelChangedAtStep(float PreviousFuelKg);

	bool bLogFuel = true;
};
