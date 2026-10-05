#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "SpaceNavThrusterVisualComponent.generated.h"

class UPointLightComponent;
class USceneComponent;
class UStaticMesh;
class UStaticMeshComponent;

UCLASS(BlueprintType, Blueprintable, ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class SPACENAVSIM_API USpaceNavThrusterVisualComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	USpaceNavThrusterVisualComponent();

	UFUNCTION(BlueprintCallable, Category = "SpaceNavThrusters|Visuals")
	void InitializeThrusters(USceneComponent* ForwardPoint,
		USceneComponent* StrafeLeftPoint, USceneComponent* StrafeRightPoint,
		USceneComponent* StrafeUpPoint, USceneComponent* StrafeDownPoint,
		USceneComponent* YawLeftPoint, USceneComponent* YawRightPoint,
		USceneComponent* PitchUpPoint, USceneComponent* PitchDownPoint);

	UFUNCTION(BlueprintCallable, Category = "SpaceNavThrusters|Visuals")
	void StartForward();

	UFUNCTION(BlueprintCallable, Category = "SpaceNavThrusters|Visuals")
	void StartStrafeLeft();

	UFUNCTION(BlueprintCallable, Category = "SpaceNavThrusters|Visuals")
	void StartStrafeRight();

	UFUNCTION(BlueprintCallable, Category = "SpaceNavThrusters|Visuals")
	void StartStrafeUp();

	UFUNCTION(BlueprintCallable, Category = "SpaceNavThrusters|Visuals")
	void StartStrafeDown();

	UFUNCTION(BlueprintCallable, Category = "SpaceNavThrusters|Visuals")
	void StartYawLeft();

	UFUNCTION(BlueprintCallable, Category = "SpaceNavThrusters|Visuals")
	void StartYawRight();

	UFUNCTION(BlueprintCallable, Category = "SpaceNavThrusters|Visuals")
	void StartPitchUp();

	UFUNCTION(BlueprintCallable, Category = "SpaceNavThrusters|Visuals")
	void StartPitchDown();

	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings|VFX")
	TObjectPtr<UStaticMesh> FlameMesh;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings|VFX")
	float ForwardScaleMultiplier = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings|VFX")
	float StrafeLeftScaleMultiplier = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings|VFX")
	float StrafeRightScaleMultiplier = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings|VFX")
	float StrafeUpScaleMultiplier = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings|VFX")
	float StrafeDownScaleMultiplier = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings|VFX")
	float YawLeftScaleMultiplier = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings|VFX")
	float YawRightScaleMultiplier = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings|VFX")
	float PitchUpScaleMultiplier = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings|VFX")
	float PitchDownScaleMultiplier = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings|VFX",
		meta = (ToolTip = "Seconds to grow from zero to the configured flame size."))
	float FadeInSeconds = 0.05f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings|VFX",
		meta = (ToolTip = "Seconds to shrink from the configured flame size to zero."))
	float FadeOutSeconds = 0.1f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings|Light")
	FLinearColor LightColor = FLinearColor::Red;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings|Light",
		meta = (ToolTip = "Point light intensity when the flame is fully visible."))
	float LightIntensity = 5000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings|Light",
		meta = (ToolTip = "Point light attenuation radius in Unreal units."))
	float LightRadius = 1000.0f;

private:
	void InitializeVisuals(const TArray<USceneComponent*>& Points);
	void StartThruster(int32 ThrusterIndex);
	void UpdateThrusters(float DeltaTime);
	void DestroyVisuals();

	bool CanInitialize(const TArray<USceneComponent*>& Points) const;
	void CreateVisuals(const TArray<USceneComponent*>& Points);
	void UpdateThruster(int32 ThrusterIndex, float DeltaTime);
	void ApplyVisuals(int32 ThrusterIndex);
	float GetScaleMultiplier(int32 ThrusterIndex) const;

	static constexpr int32 ThrusterCount = 9;

	UPROPERTY(Transient)
	TArray<TObjectPtr<UStaticMeshComponent>> FlameComponents;

	UPROPERTY(Transient)
	TArray<TObjectPtr<UPointLightComponent>> LightComponents;

	float FlameLevels[ThrusterCount] = {};
	bool bGrowing[ThrusterCount] = {};
	bool bTriggered[ThrusterCount] = {};
};