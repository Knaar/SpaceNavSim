#include "Game/Pawn/SpaceNavPawn.h"

#include "Camera/CameraComponent.h"
#include "Components/InputComponent.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/Controller.h"
#include "GameFramework/FloatingPawnMovement.h"
#include "GameFramework/SpringArmComponent.h"
#include "UObject/ConstructorHelpers.h"

DEFINE_LOG_CATEGORY_STATIC(LogSpaceNavPawn, Log, All);

ASpaceNavPawn::ASpaceNavPawn()
{
	SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("SceneRoot"));
	SetRootComponent(SceneRoot);

	ShipMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("ShipMesh"));
	ShipMesh->SetupAttachment(SceneRoot);
	ShipMesh->SetRelativeRotation(FRotator(90.0f, 0.0f, 0.0f));
	ShipMesh->SetRelativeScale3D(FVector(0.5, 0.5, 1.0));
	ShipMesh->SetCastShadow(false);
	static ConstructorHelpers::FObjectFinder<UStaticMesh> shipMeshAsset(TEXT("/Engine/BasicShapes/Cone.Cone"));
	if (shipMeshAsset.Succeeded()) ShipMesh->SetStaticMesh(shipMeshAsset.Object);

	CameraBoom = CreateDefaultSubobject<USpringArmComponent>(TEXT("CameraBoom"));
	CameraBoom->SetupAttachment(ShipMesh);
	CameraBoom->SetAbsolute(false, false, true);
	CameraBoom->TargetArmLength = 300.0f;
	CameraBoom->bUsePawnControlRotation = true;
	CameraBoom->bDoCollisionTest = false;

	Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
	Camera->SetupAttachment(CameraBoom, USpringArmComponent::SocketName);
	Camera->bUsePawnControlRotation = false;

	MovementComponent = CreateDefaultSubobject<UFloatingPawnMovement>(TEXT("MovementComponent"));

	bUseControllerRotationPitch = false;
	bUseControllerRotationYaw = false;
	bUseControllerRotationRoll = false;
}

void ASpaceNavPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	if (!bInputSetupLogged && Controller != nullptr)
	{
		const FRotator currentRotation = Controller->GetControlRotation();
		Controller->SetControlRotation(FRotator(-15.0f, currentRotation.Yaw, 0.0f));
	}

	if (!bInputSetupLogged) LogInput(TEXT("Started"));
	PlayerInputComponent->BindAxis(TEXT("LookYaw"), this, &ASpaceNavPawn::LookYaw);
	PlayerInputComponent->BindAxis(TEXT("LookPitch"), this, &ASpaceNavPawn::LookPitch);
	if (!bInputSetupLogged) LogInput(TEXT("Controls ready"));
	bInputSetupLogged = true;
}

UPawnMovementComponent* ASpaceNavPawn::GetMovementComponent() const
{
	return MovementComponent.Get();
}

void ASpaceNavPawn::LookYaw(float Value)
{
	if (FMath::IsNearlyZero(Value)) return;
	AddControllerYawInput(Value);
}

void ASpaceNavPawn::LookPitch(float Value)
{
	if (FMath::IsNearlyZero(Value)) return;
	AddControllerPitchInput(Value);
}

void ASpaceNavPawn::LogInput(const TCHAR* Message) const
{
	if (!bLogInput) return;
	UE_LOG(LogSpaceNavPawn, Display, TEXT("%s: %s"), *GetNameSafe(this), Message);
}
