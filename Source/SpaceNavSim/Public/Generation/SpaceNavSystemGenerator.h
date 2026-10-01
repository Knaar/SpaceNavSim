#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SpaceNavSystemGenerator.generated.h"

class USpaceNavSystemGeneratorDataAsset;
class ASpaceNavCentralBody;
class UMaterialInterface;
struct FRandomStream;

UCLASS(PrioritizeCategories = "Settings|Data")
class SPACENAVSIM_API ASpaceNavSystemGenerator : public AActor
{
	GENERATED_BODY()

public:
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
	TArray<double> GeneratePlanetElevations(FRandomStream& RandomStream) const;
	bool SpawnPlanet(FRandomStream& RandomStream, double ElevationDegrees, bool& bWarnedMissingMaterial);
	UMaterialInterface* SelectPlanetMaterial(FRandomStream& RandomStream, bool& bWarnedMissingMaterial) const;
	void LogGeneration(ELogVerbosity::Type Verbosity, const TCHAR* Message) const;

	bool bLogGeneration = true;
};
