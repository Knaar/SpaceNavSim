#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "SpaceNavUIComponent.generated.h"

class USpaceNavMainWidget;

UCLASS(Blueprintable, ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class SPACENAVSIM_API USpaceNavUIComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UFUNCTION(BlueprintCallable, Category = "SpaceNavUIComponent|Initialization")
	void InitializeMainWidget();

	UFUNCTION(BlueprintCallable, Category = "SpaceNavUIComponent|Updates")
	void UpdateCurrentMass(float Value);

	UFUNCTION(BlueprintCallable, Category = "SpaceNavUIComponent|Updates")
	void UpdateCurrentSpeed(float Value);

	UFUNCTION(BlueprintCallable, Category = "SpaceNavUIComponent|Updates")
	void UpdateRemainingFuel(float Value);

	UFUNCTION(BlueprintCallable, Category = "SpaceNavUIComponent|Updates")
	void UpdateFuelConsumption(float Value);

	UFUNCTION(BlueprintCallable, Category = "SpaceNavUIComponent|Feedback")
	void PressFuelEfficient();

	UFUNCTION(BlueprintCallable, Category = "SpaceNavUIComponent|Feedback")
	void PressFast();

	UFUNCTION(BlueprintCallable, Category = "SpaceNavUIComponent|Feedback")
	void PressBalanced();

	UFUNCTION(BlueprintCallable, Category = "SpaceNavUIComponent|Feedback")
	void PressEngines();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings|UI")
	TSubclassOf<USpaceNavMainWidget> MainWidgetClass;

	UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly, Category = "SpaceNavUIComponent|Runtime")
	TObjectPtr<USpaceNavMainWidget> MainWidget;
};
