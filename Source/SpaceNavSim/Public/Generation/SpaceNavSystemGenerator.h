#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SpaceNavSystemGenerator.generated.h"

class USpaceNavSystemGeneratorDataAsset;
class ASpaceNavCentralBody;
class ASpaceNavStartMarker;
class UMaterialInterface;
struct FRandomStream;

DECLARE_MULTICAST_DELEGATE(FOnSpaceNavStartZoneReady);

UCLASS(PrioritizeCategories = "Settings|Data")
class SPACENAVSIM_API ASpaceNavSystemGenerator : public AActor
{
	GENERATED_BODY()

public:
	ASpaceNavStartMarker* GetStartMarker() const;
	FOnSpaceNavStartZoneReady OnStartZoneReady;

	UPROPERTY(EditAnywhere, Category = "Settings|Data")
	TObjectPtr<USpaceNavSystemGeneratorDataAsset> GeneratorData;

protected:
	virtual void BeginPlay() override;

private:
	bool PrepareGeneration();
	void GenerateSystem();

	void LoadLoggingConfig();
	bool CheckGeneratorData() const;
	bool ValidateSettings() const;
	bool ValidateSystemSettings() const;
	bool ValidateCentralBodySettings() const;
	bool ValidatePlanetSettings() const;
	bool ValidatePlanetRanges() const;
	ASpaceNavCentralBody* SpawnCentralBody();
	int32 SpawnPlanets();
	bool SpawnMarkers();
	TArray<FVector> GenerateOrbitSlots() const;
	bool SpawnPlanet(FRandomStream& RandomStream, const FVector& LocalOrbitPositionKm,
		double SignedAngularSpeedRadiansPerSecond, bool& bWarnedMissingMaterial);
	UMaterialInterface* SelectPlanetMaterial(FRandomStream& RandomStream, bool& bWarnedMissingMaterial) const;
	void LogGeneration(ELogVerbosity::Type Verbosity, const TCHAR* Message) const;

	UPROPERTY(Transient)
	TObjectPtr<ASpaceNavStartMarker> StartMarker;

	bool bLogGeneration = true;
};
