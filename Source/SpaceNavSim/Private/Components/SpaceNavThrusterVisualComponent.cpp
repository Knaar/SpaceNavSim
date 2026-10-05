#include "Components/SpaceNavThrusterVisualComponent.h"

#include "Components/PointLightComponent.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/Actor.h"

USpaceNavThrusterVisualComponent::USpaceNavThrusterVisualComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = false;
}

void USpaceNavThrusterVisualComponent::InitializeThrusters(USceneComponent* ForwardPoint,
	USceneComponent* StrafeLeftPoint, USceneComponent* StrafeRightPoint,
	USceneComponent* StrafeUpPoint, USceneComponent* StrafeDownPoint,
	USceneComponent* YawLeftPoint, USceneComponent* YawRightPoint,
	USceneComponent* PitchUpPoint, USceneComponent* PitchDownPoint)
{
	InitializeVisuals({ForwardPoint, StrafeLeftPoint, StrafeRightPoint,
		StrafeUpPoint, StrafeDownPoint, YawLeftPoint, YawRightPoint,
		PitchUpPoint, PitchDownPoint});
}

void USpaceNavThrusterVisualComponent::StartForward()
{
	StartThruster(0);
}

void USpaceNavThrusterVisualComponent::StartStrafeLeft()
{
	StartThruster(1);
}

void USpaceNavThrusterVisualComponent::StartStrafeRight()
{
	StartThruster(2);
}

void USpaceNavThrusterVisualComponent::StartStrafeUp()
{
	StartThruster(3);
}

void USpaceNavThrusterVisualComponent::StartStrafeDown()
{
	StartThruster(4);
}

void USpaceNavThrusterVisualComponent::StartYawLeft()
{
	StartThruster(5);
}

void USpaceNavThrusterVisualComponent::StartYawRight()
{
	StartThruster(6);
}

void USpaceNavThrusterVisualComponent::StartPitchUp()
{
	StartThruster(7);
}

void USpaceNavThrusterVisualComponent::StartPitchDown()
{
	StartThruster(8);
}

void USpaceNavThrusterVisualComponent::TickComponent(float DeltaTime, ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	UpdateThrusters(DeltaTime);
}

void USpaceNavThrusterVisualComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	DestroyVisuals();
	Super::EndPlay(EndPlayReason);
}

void USpaceNavThrusterVisualComponent::InitializeVisuals(const TArray<USceneComponent*>& Points)
{
	if (!CanInitialize(Points)) return;
	DestroyVisuals();
	CreateVisuals(Points);
}

void USpaceNavThrusterVisualComponent::StartThruster(int32 ThrusterIndex)
{
	if (!FlameComponents.IsValidIndex(ThrusterIndex)) return;
	if (!IsValid(FlameComponents[ThrusterIndex].Get())) return;
	if (FlameLevels[ThrusterIndex] < 1.0f) bGrowing[ThrusterIndex] = true;
	bTriggered[ThrusterIndex] = true;
	SetComponentTickEnabled(true);
}

void USpaceNavThrusterVisualComponent::UpdateThrusters(float DeltaTime)
{
	bool bAnyActive = false;
	for (int32 thrusterIndex = 0; thrusterIndex < FlameComponents.Num(); ++thrusterIndex)
	{
		UpdateThruster(thrusterIndex, DeltaTime);
		ApplyVisuals(thrusterIndex);
		bAnyActive |= FlameLevels[thrusterIndex] > 0.0f;
	}
	if (!bAnyActive) SetComponentTickEnabled(false);
}

void USpaceNavThrusterVisualComponent::DestroyVisuals()
{
	SetComponentTickEnabled(false);
	AActor* owner = GetOwner();
	for (UStaticMeshComponent* flame : FlameComponents)
	{
		if (!IsValid(flame)) continue;
		if (owner != nullptr) owner->RemoveInstanceComponent(flame);
		flame->DestroyComponent();
	}
	for (UPointLightComponent* light : LightComponents)
	{
		if (!IsValid(light)) continue;
		if (owner != nullptr) owner->RemoveInstanceComponent(light);
		light->DestroyComponent();
	}
	FlameComponents.Empty();
	LightComponents.Empty();
	for (int32 thrusterIndex = 0; thrusterIndex < ThrusterCount; ++thrusterIndex)
	{
		FlameLevels[thrusterIndex] = 0.0f;
		bGrowing[thrusterIndex] = false;
		bTriggered[thrusterIndex] = false;
	}
}

bool USpaceNavThrusterVisualComponent::CanInitialize(const TArray<USceneComponent*>& Points) const
{
	if (GetOwner() == nullptr || !IsValid(FlameMesh.Get()) || Points.Num() != ThrusterCount) return false;
	for (USceneComponent* point : Points)
		if (!IsValid(point)) return false;
	return true;
}

void USpaceNavThrusterVisualComponent::CreateVisuals(const TArray<USceneComponent*>& Points)
{
	AActor* owner = GetOwner();
	for (USceneComponent* point : Points)
	{
		UStaticMeshComponent* flame = NewObject<UStaticMeshComponent>(owner, NAME_None, RF_Transient);
		flame->SetMobility(EComponentMobility::Movable);
		flame->SetupAttachment(point);
		flame->SetStaticMesh(FlameMesh);
		flame->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		flame->SetGenerateOverlapEvents(false);
		flame->SetCanEverAffectNavigation(false);
		flame->SetRelativeScale3D(FVector::ZeroVector);
		flame->SetVisibility(false);
		owner->AddInstanceComponent(flame);
		flame->RegisterComponent();
		FlameComponents.Add(flame);

		UPointLightComponent* light = NewObject<UPointLightComponent>(owner, NAME_None, RF_Transient);
		light->SetMobility(EComponentMobility::Movable);
		light->SetupAttachment(point);
		light->SetLightColor(LightColor);
		light->SetAttenuationRadius(LightRadius);
		light->SetIntensity(0.0f);
		light->SetVisibility(false);
		owner->AddInstanceComponent(light);
		light->RegisterComponent();
		LightComponents.Add(light);
	}
}

void USpaceNavThrusterVisualComponent::UpdateThruster(int32 ThrusterIndex, float DeltaTime)
{
	float& flameLevel = FlameLevels[ThrusterIndex];
	if (bGrowing[ThrusterIndex])
	{
		flameLevel = FadeInSeconds > 0.0f
			? FMath::Min(1.0f, flameLevel + DeltaTime / FadeInSeconds) : 1.0f;
		if (flameLevel >= 1.0f) bGrowing[ThrusterIndex] = false;
	}
	else if (!bTriggered[ThrusterIndex])
	{
		flameLevel = FadeOutSeconds > 0.0f
			? FMath::Max(0.0f, flameLevel - DeltaTime / FadeOutSeconds) : 0.0f;
	}
	bTriggered[ThrusterIndex] = false;
}

void USpaceNavThrusterVisualComponent::ApplyVisuals(int32 ThrusterIndex)
{
	UStaticMeshComponent* flame = FlameComponents[ThrusterIndex];
	UPointLightComponent* light = LightComponents[ThrusterIndex];
	if (!IsValid(flame) || !IsValid(light)) return;
	const float flameLevel = FlameLevels[ThrusterIndex];
	const float scale = GetScaleMultiplier(ThrusterIndex) * flameLevel;
	const bool bVisible = scale != 0.0f;
	flame->SetRelativeScale3D(FVector(scale));
	flame->SetVisibility(bVisible);
	light->SetIntensity(bVisible ? LightIntensity * flameLevel : 0.0f);
	light->SetVisibility(bVisible);
}

float USpaceNavThrusterVisualComponent::GetScaleMultiplier(int32 ThrusterIndex) const
{
	switch (ThrusterIndex)
	{
	case 0: return ForwardScaleMultiplier;
	case 1: return StrafeLeftScaleMultiplier;
	case 2: return StrafeRightScaleMultiplier;
	case 3: return StrafeUpScaleMultiplier;
	case 4: return StrafeDownScaleMultiplier;
	case 5: return YawLeftScaleMultiplier;
	case 6: return YawRightScaleMultiplier;
	case 7: return PitchUpScaleMultiplier;
	case 8: return PitchDownScaleMultiplier;
	default: return 0.0f;
	}
}