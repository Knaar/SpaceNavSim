#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "SpaceNavPawn.generated.h"

class UCameraComponent;
class UFloatingPawnMovement;
class USceneComponent;
class USpringArmComponent;
class UStaticMeshComponent;

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
	TObjectPtr<UStaticMeshComponent> ShipMesh;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SpaceNavPawn|Components")
	TObjectPtr<USpringArmComponent> CameraBoom;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SpaceNavPawn|Components")
	TObjectPtr<UCameraComponent> Camera;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SpaceNavPawn|Components")
	TObjectPtr<UFloatingPawnMovement> MovementComponent;

private:
	void MoveForward(float Value);
	void MoveRight(float Value);
	void LookYaw(float Value);
	void LookPitch(float Value);
	void LogInput(const TCHAR* Message) const;

	UPROPERTY(EditAnywhere, Category = "Settings|Debug")
	bool bLogInput = true;

	bool bInputSetupLogged = false;
};
