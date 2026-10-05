#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "SpaceNavPawn.generated.h"

class UCameraComponent;
class UFloatingPawnMovement;
class USceneComponent;
class USpringArmComponent;
class UStaticMeshComponent;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FSpaceNavFuelRequest, float, SuggestedFuelAmount);

UCLASS(PrioritizeCategories = "Settings|Debug")
class SPACENAVSIM_API ASpaceNavPawn : public APawn
{
	GENERATED_BODY()

public:
	ASpaceNavPawn();

	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;
	virtual UPawnMovementComponent* GetMovementComponent() const override;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SpaceNavPawn|Components")
	TObjectPtr<USceneComponent> SceneRoot;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SpaceNavPawn|Components")
	TObjectPtr<USceneComponent> ForwardThrusterPoint;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SpaceNavPawn|Components")
	TObjectPtr<USceneComponent> StrafeLeftThrusterPoint;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SpaceNavPawn|Components")
	TObjectPtr<USceneComponent> StrafeRightThrusterPoint;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SpaceNavPawn|Components")
	TObjectPtr<USceneComponent> StrafeUpThrusterPoint;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SpaceNavPawn|Components")
	TObjectPtr<USceneComponent> StrafeDownThrusterPoint;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SpaceNavPawn|Components")
	TObjectPtr<USceneComponent> YawLeftThrusterPoint;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SpaceNavPawn|Components")
	TObjectPtr<USceneComponent> YawRightThrusterPoint;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SpaceNavPawn|Components")
	TObjectPtr<USceneComponent> PitchUpThrusterPoint;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SpaceNavPawn|Components")
	TObjectPtr<USceneComponent> PitchDownThrusterPoint;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SpaceNavPawn|Components")
	TObjectPtr<UStaticMeshComponent> ShipMesh;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SpaceNavPawn|Components")
	TObjectPtr<USpringArmComponent> CameraBoom;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SpaceNavPawn|Components")
	TObjectPtr<UCameraComponent> Camera;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SpaceNavPawn|Components")
	TObjectPtr<UFloatingPawnMovement> MovementComponent;

	UPROPERTY(BlueprintAssignable, Category = "SpaceNavPawn|Events")
	FSpaceNavFuelRequest OnMoveForward;

	UPROPERTY(BlueprintAssignable, Category = "SpaceNavPawn|Events")
	FSpaceNavFuelRequest OnMoveLeft;

	UPROPERTY(BlueprintAssignable, Category = "SpaceNavPawn|Events")
	FSpaceNavFuelRequest OnMoveRight;

	UPROPERTY(BlueprintAssignable, Category = "SpaceNavPawn|Events")
	FSpaceNavFuelRequest OnMoveUp;

	UPROPERTY(BlueprintAssignable, Category = "SpaceNavPawn|Events")
	FSpaceNavFuelRequest OnMoveDown;

	UPROPERTY(BlueprintAssignable, Category = "SpaceNavPawn|Events")
	FSpaceNavFuelRequest OnTurnYawLeft;

	UPROPERTY(BlueprintAssignable, Category = "SpaceNavPawn|Events")
	FSpaceNavFuelRequest OnTurnYawRight;

	UPROPERTY(BlueprintAssignable, Category = "SpaceNavPawn|Events")
	FSpaceNavFuelRequest OnTurnPitchUp;

	UPROPERTY(BlueprintAssignable, Category = "SpaceNavPawn|Events")
	FSpaceNavFuelRequest OnTurnPitchDown;

private:
	void LookYaw(float Value);
	void LookPitch(float Value);
	void LogInput(const TCHAR* Message) const;

	UPROPERTY(EditAnywhere, Category = "Settings|Debug")
	bool bLogInput = true;

	bool bInputSetupLogged = false;
};
