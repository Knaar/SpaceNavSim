#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "SpaceNavEngineComponent.generated.h"

class USpaceNavPawnSettingsDataAsset;

UCLASS(BlueprintType, Blueprintable, ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class SPACENAVSIM_API USpaceNavEngineComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	USpaceNavEngineComponent();

	UFUNCTION(BlueprintCallable, Category = "SpaceNavEngine|Movement")
	bool InitEngine();

	void BeginScriptedFlight();
	void SetScriptedVelocity(const FVector& WorldVelocity);
	void EndScriptedFlight(const FVector& ExitVelocity);

	UFUNCTION(BlueprintCallable, Category = "SpaceNavEngine|Movement")
	void EnginePush(float FuelAmount);

	UFUNCTION(BlueprintCallable, Category = "SpaceNavEngine|Movement")
	void EnginePushLeft(float FuelAmount);

	UFUNCTION(BlueprintCallable, Category = "SpaceNavEngine|Movement")
	void EnginePushRight(float FuelAmount);

	UFUNCTION(BlueprintCallable, Category = "SpaceNavEngine|Movement")
	void EnginePushUp(float FuelAmount);

	UFUNCTION(BlueprintCallable, Category = "SpaceNavEngine|Movement")
	void EnginePushDown(float FuelAmount);

	UFUNCTION(BlueprintCallable, Category = "SpaceNavEngine|Rotation")
	void EngineTurnYawLeft(float FuelAmount);

	UFUNCTION(BlueprintCallable, Category = "SpaceNavEngine|Rotation")
	void EngineTurnYawRight(float FuelAmount);

	UFUNCTION(BlueprintCallable, Category = "SpaceNavEngine|Rotation")
	void EngineTurnPitchUp(float FuelAmount);

	UFUNCTION(BlueprintCallable, Category = "SpaceNavEngine|Rotation")
	void EngineTurnPitchDown(float FuelAmount);

	FVector GetLinearVelocity() const { return Velocity; }
	FVector GetGravityVelocity() const { return GravityVelocity; }
	FVector GetTotalLinearVelocity() const { return Velocity + GravityVelocity; }
	float GetYawAngularVelocity() const { return YawAngularVelocity; }
	float GetPitchAngularVelocity() const { return PitchAngularVelocity; }
	static FVector CalculateBodyGravityAcceleration(const FVector& PawnLocation,
		const FVector& BodyLocation, double MassTonnes, double RadiusMeters,
		double GravityCoefficient);

	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;

	UPROPERTY(EditAnywhere, Category = "Settings|Data")
	TObjectPtr<USpaceNavPawnSettingsDataAsset> PawnSettings;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings|Movement")
	float ThrustAcceleration = 2000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings|Movement")
	float MaxSpeed = 10000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings|Rotation")
	float TurnImpulsePerFuelUnit = 90.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings|Rotation")
	float TurnDeceleration = 180.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings|Rotation")
	float MaxTurnSpeed = 180.0f;

private:
	FVector Velocity = FVector::ZeroVector;
	FVector GravityVelocity = FVector::ZeroVector;
	float YawAngularVelocity = 0.0f;
	float PitchAngularVelocity = 0.0f;
	bool bScriptedFlight = false;
	bool bSkipNextPhysicsStep = false;
};
