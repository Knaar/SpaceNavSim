#include "Components/SpaceNavEngineComponent.h"

#include "EngineUtils.h"
#include "Game/Pawn/SpaceNavPawn.h"
#include "Game/Pawn/SpaceNavPawnSettingsDataAsset.h"
#include "GameFramework/Actor.h"
#include "Generation/SpaceNavSystemGenerator.h"
#include "Generation/SpaceNavSystemGeneratorDataAsset.h"
#include "InteractiveObjects/Actors/SpaceNavCentralBody.h"
#include "InteractiveObjects/Actors/SpaceNavPlanet.h"

namespace
{
	constexpr double EngineUnitsPerMeter = 100.0;

	FVector CalculateGravityAcceleration(const ASpaceNavPawn& Pawn)
	{
		UWorld* world = Pawn.GetWorld();
		if (world == nullptr) return FVector::ZeroVector;

		double gravityCoefficient = 0.0;
		for (TActorIterator<ASpaceNavSystemGenerator> generatorIterator(world); generatorIterator; ++generatorIterator)
		{
			if (generatorIterator->GeneratorData == nullptr) continue;
			gravityCoefficient = generatorIterator->GeneratorData->GravityCoefficient;
			break;
		}
		if (gravityCoefficient <= 0.0 || !FMath::IsFinite(gravityCoefficient)) return FVector::ZeroVector;

		FVector totalAcceleration = FVector::ZeroVector;
		for (TActorIterator<ASpaceNavPlanet> planetIterator(world); planetIterator; ++planetIterator)
		{
			totalAcceleration += USpaceNavEngineComponent::CalculateBodyGravityAcceleration(
				Pawn.GetActorLocation(), planetIterator->GetActorLocation(), planetIterator->MassTonnes,
				planetIterator->RadiusMeters, gravityCoefficient);
		}
		for (TActorIterator<ASpaceNavCentralBody> bodyIterator(world); bodyIterator; ++bodyIterator)
		{
			totalAcceleration += USpaceNavEngineComponent::CalculateBodyGravityAcceleration(
				Pawn.GetActorLocation(), bodyIterator->GetActorLocation(), bodyIterator->MassTonnes,
				bodyIterator->RadiusMeters, gravityCoefficient);
		}
		return totalAcceleration;
	}
}

USpaceNavEngineComponent::USpaceNavEngineComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
}

FVector USpaceNavEngineComponent::CalculateBodyGravityAcceleration(const FVector& PawnLocation,
	const FVector& BodyLocation, double MassTonnes, double RadiusMeters,
	double GravityCoefficient)
{
	const FVector towardBody = BodyLocation - PawnLocation;
	const double centerDistanceUU = towardBody.Size();
	if (centerDistanceUU <= 0.0) return FVector::ZeroVector;
	const double distanceMeters = FMath::Max(centerDistanceUU / EngineUnitsPerMeter, RadiusMeters);
	const double accelerationUU = EngineUnitsPerMeter * GravityCoefficient * MassTonnes /
		FMath::Square(distanceMeters);
	return towardBody / centerDistanceUU * accelerationUU;
}

bool USpaceNavEngineComponent::InitEngine()
{
	if (PawnSettings == nullptr) return false;
	ThrustAcceleration = PawnSettings->MaxThrust;
	MaxSpeed = PawnSettings->MaxSpeed;
	return true;
}

void USpaceNavEngineComponent::SetInitialVelocity(const FVector& WorldVelocity)
{
	Velocity = WorldVelocity;
}

void USpaceNavEngineComponent::BeginScriptedFlight()
{
	bScriptedFlight = true;
	bSkipNextPhysicsStep = false;
	Velocity = FVector::ZeroVector;
	GravityVelocity = FVector::ZeroVector;
	YawAngularVelocity = 0.0f;
	PitchAngularVelocity = 0.0f;
}

void USpaceNavEngineComponent::SetScriptedVelocity(const FVector& WorldVelocity)
{
	if (!bScriptedFlight) return;
	Velocity = WorldVelocity;
	GravityVelocity = FVector::ZeroVector;
	YawAngularVelocity = 0.0f;
	PitchAngularVelocity = 0.0f;
}

void USpaceNavEngineComponent::EndScriptedFlight(const FVector& ExitVelocity)
{
	Velocity = ExitVelocity;
	GravityVelocity = FVector::ZeroVector;
	YawAngularVelocity = 0.0f;
	PitchAngularVelocity = 0.0f;
	bScriptedFlight = false;
	bSkipNextPhysicsStep = true;
}

void USpaceNavEngineComponent::EnginePush(float FuelAmount)
{
	if (bScriptedFlight) return;
	Velocity += GetOwner()->GetActorForwardVector() * (ThrustAcceleration * FuelAmount);
	Velocity = Velocity.GetClampedToMaxSize(MaxSpeed);
}

void USpaceNavEngineComponent::EnginePushLeft(float FuelAmount)
{
	if (bScriptedFlight || FuelAmount <= 0.0f) return;
	Velocity -= GetOwner()->GetActorRightVector() * (ThrustAcceleration * FuelAmount);
	Velocity = Velocity.GetClampedToMaxSize(MaxSpeed);
}

void USpaceNavEngineComponent::EnginePushRight(float FuelAmount)
{
	if (bScriptedFlight || FuelAmount <= 0.0f) return;
	Velocity += GetOwner()->GetActorRightVector() * (ThrustAcceleration * FuelAmount);
	Velocity = Velocity.GetClampedToMaxSize(MaxSpeed);
}

void USpaceNavEngineComponent::EnginePushUp(float FuelAmount)
{
	if (bScriptedFlight || FuelAmount <= 0.0f) return;
	Velocity += GetOwner()->GetActorUpVector() * (ThrustAcceleration * FuelAmount);
	Velocity = Velocity.GetClampedToMaxSize(MaxSpeed);
}

void USpaceNavEngineComponent::EnginePushDown(float FuelAmount)
{
	if (bScriptedFlight || FuelAmount <= 0.0f) return;
	Velocity -= GetOwner()->GetActorUpVector() * (ThrustAcceleration * FuelAmount);
	Velocity = Velocity.GetClampedToMaxSize(MaxSpeed);
}

void USpaceNavEngineComponent::EngineTurnYawLeft(float FuelAmount)
{
	if (bScriptedFlight || FuelAmount <= 0.0f) return;
	YawAngularVelocity = FMath::Clamp(YawAngularVelocity - TurnImpulsePerFuelUnit * FuelAmount,
		-MaxTurnSpeed, MaxTurnSpeed);
}

void USpaceNavEngineComponent::EngineTurnYawRight(float FuelAmount)
{
	if (bScriptedFlight || FuelAmount <= 0.0f) return;
	YawAngularVelocity = FMath::Clamp(YawAngularVelocity + TurnImpulsePerFuelUnit * FuelAmount,
		-MaxTurnSpeed, MaxTurnSpeed);
}

void USpaceNavEngineComponent::EngineTurnPitchUp(float FuelAmount)
{
	if (bScriptedFlight || FuelAmount <= 0.0f) return;
	PitchAngularVelocity = FMath::Clamp(PitchAngularVelocity + TurnImpulsePerFuelUnit * FuelAmount,
		-MaxTurnSpeed, MaxTurnSpeed);
}

void USpaceNavEngineComponent::EngineTurnPitchDown(float FuelAmount)
{
	if (bScriptedFlight || FuelAmount <= 0.0f) return;
	PitchAngularVelocity = FMath::Clamp(PitchAngularVelocity - TurnImpulsePerFuelUnit * FuelAmount,
		-MaxTurnSpeed, MaxTurnSpeed);
}

void USpaceNavEngineComponent::TickComponent(float DeltaTime, ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (bScriptedFlight) return;
	if (bSkipNextPhysicsStep)
	{
		bSkipNextPhysicsStep = false;
		return;
	}

	FVector displacement = FVector::ZeroVector;
	if (const ASpaceNavPawn* pawn = Cast<ASpaceNavPawn>(GetOwner()))
	{
		const FVector previousGravityVelocity = GravityVelocity;
		GravityVelocity += CalculateGravityAcceleration(*pawn) * DeltaTime;
		displacement += (previousGravityVelocity + GravityVelocity) * (0.5 * DeltaTime);
	}

	displacement += Velocity * DeltaTime;
	if (!displacement.IsZero()) GetOwner()->AddActorWorldOffset(displacement);

	const float nextYawAngularVelocity = FMath::FInterpConstantTo(
		YawAngularVelocity, 0.0f, DeltaTime, TurnDeceleration);
	const float nextPitchAngularVelocity = FMath::FInterpConstantTo(
		PitchAngularVelocity, 0.0f, DeltaTime, TurnDeceleration);
	const float yawDelta = (YawAngularVelocity + nextYawAngularVelocity) * 0.5f * DeltaTime;
	const float pitchDelta = (PitchAngularVelocity + nextPitchAngularVelocity) * 0.5f * DeltaTime;
	if (yawDelta != 0.0f || pitchDelta != 0.0f)
	{
		GetOwner()->AddActorLocalRotation(FRotator(pitchDelta, yawDelta, 0.0f));
	}
	YawAngularVelocity = nextYawAngularVelocity;
	PitchAngularVelocity = nextPitchAngularVelocity;
}
