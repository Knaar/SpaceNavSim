#include "Generation/SpaceNavSystemGenerator.h"

#include "Generation/SpaceNavSystemGeneratorDataAsset.h"
#include "InteractiveObjects/Actors/SpaceNavCentralBody.h"
#include "InteractiveObjects/Actors/SpaceNavPlanet.h"
#include "InteractiveObjects/Actors/SpaceNavStartMarker.h"
#include "InteractiveObjects/Actors/SpaceNavTargetMarker.h"

#include "Engine/World.h"
#include "Math/RandomStream.h"
#include "Misc/ConfigCacheIni.h"

DEFINE_LOG_CATEGORY_STATIC(LogSpaceNavSystemGenerator, Log, All);

namespace
{
	constexpr double UnrealUnitsPerKm = 100000.0;
	constexpr double MetersPerKm = 1000.0;
	constexpr double MaxOrbitalElevationDegrees = 30.0;

	bool IsValidPositiveRange(double Minimum, double Maximum);
	double SampleRange(FRandomStream& RandomStream, double Minimum, double Maximum);
}

void ASpaceNavSystemGenerator::BeginPlay()
{
	Super::BeginPlay();
	if (!PrepareGeneration()) return;
	GenerateSystem();
}

bool ASpaceNavSystemGenerator::PrepareGeneration()
{
	LoadLoggingConfig();
	LogGeneration(ELogVerbosity::Display, TEXT("Started"));
	if (!CheckGeneratorData()) return false;
	if (!ValidateSettings()) return false;
	LogGeneration(ELogVerbosity::Display, TEXT("Settings ready"));
	return true;
}

void ASpaceNavSystemGenerator::GenerateSystem()
{
	if (SpawnCentralBody() == nullptr)
	{
		LogGeneration(ELogVerbosity::Error, TEXT("Failed: body"));
		return;
	}
	LogGeneration(ELogVerbosity::Display, TEXT("Body spawned, lit"));

	const int32 spawnedPlanets = SpawnPlanets();
	if (spawnedPlanets != GeneratorData->NumberOfPlanets)
	{
		LogGeneration(ELogVerbosity::Error, *FString::Printf(TEXT("Failed: planet %d"), spawnedPlanets + 1));
		return;
	}
	LogGeneration(ELogVerbosity::Display, *FString::Printf(TEXT("Planets %d"), spawnedPlanets));
	if (!SpawnMarkers())
	{
		LogGeneration(ELogVerbosity::Error, TEXT("Failed: markers"));
		return;
	}
	LogGeneration(ELogVerbosity::Display, TEXT("Markers 2/2"));
	LogGeneration(ELogVerbosity::Display, TEXT("Done"));
}

void ASpaceNavSystemGenerator::LoadLoggingConfig()
{
	GConfig->GetBool(TEXT("SpaceNavSystemGenerator"), TEXT("bLogGeneration"), bLogGeneration, GGameIni);
}

bool ASpaceNavSystemGenerator::CheckGeneratorData() const
{
	if (GeneratorData == nullptr)
	{
		LogGeneration(ELogVerbosity::Error, TEXT("Generator data missing"));
		return false;
	}
	return true;
}

bool ASpaceNavSystemGenerator::ValidateSettings() const
{
	return ValidateSystemSettings() && ValidateCentralBodySettings() && ValidatePlanetSettings();
}

bool ASpaceNavSystemGenerator::ValidateSystemSettings() const
{
	if (!FMath::IsFinite(GeneratorData->GravityCoefficient) || GeneratorData->GravityCoefficient <= 0.0)
	{
		LogGeneration(ELogVerbosity::Error, TEXT("Gravity coeff invalid"));
		return false;
	}
	if (!FMath::IsFinite(GeneratorData->SystemBoundaryKm) || GeneratorData->SystemBoundaryKm <= 0.0)
	{
		LogGeneration(ELogVerbosity::Error, TEXT("Boundary invalid"));
		return false;
	}
	return true;
}

bool ASpaceNavSystemGenerator::ValidateCentralBodySettings() const
{
	if (GeneratorData->CentralBodyClass == nullptr)
	{
		LogGeneration(ELogVerbosity::Error, TEXT("Body class missing"));
		return false;
	}
	if (!FMath::IsFinite(GeneratorData->CentralBodyMassTonnes) || GeneratorData->CentralBodyMassTonnes <= 0.0)
	{
		LogGeneration(ELogVerbosity::Error, TEXT("Body mass invalid"));
		return false;
	}
	if (!FMath::IsFinite(GeneratorData->CentralBodyRadiusMeters) || GeneratorData->CentralBodyRadiusMeters <= 0.0)
	{
		LogGeneration(ELogVerbosity::Error, TEXT("Body radius invalid"));
		return false;
	}
	return true;
}

bool ASpaceNavSystemGenerator::ValidatePlanetSettings() const
{
	if (GeneratorData->NumberOfPlanets < 0)
	{
		LogGeneration(ELogVerbosity::Error, TEXT("Planet count invalid"));
		return false;
	}
	if (GeneratorData->NumberOfPlanets == 0)
	{
		return true;
	}
	if (GeneratorData->PlanetClass == nullptr)
	{
		LogGeneration(ELogVerbosity::Error, TEXT("Planet class missing"));
		return false;
	}
	return ValidatePlanetRanges();
}

bool ASpaceNavSystemGenerator::ValidatePlanetRanges() const
{
	if (!IsValidPositiveRange(GeneratorData->MinPlanetMassTonnes, GeneratorData->MaxPlanetMassTonnes))
	{
		LogGeneration(ELogVerbosity::Error, TEXT("Planet mass invalid"));
		return false;
	}
	if (!IsValidPositiveRange(GeneratorData->MinPlanetRadiusMeters, GeneratorData->MaxPlanetRadiusMeters))
	{
		LogGeneration(ELogVerbosity::Error, TEXT("Planet radius invalid"));
		return false;
	}
	if (!IsValidPositiveRange(GeneratorData->MinOrbitalRadiusKm, GeneratorData->MaxOrbitalRadiusKm) ||
		GeneratorData->MaxOrbitalRadiusKm > GeneratorData->SystemBoundaryKm)
	{
		LogGeneration(ELogVerbosity::Error, TEXT("Orbit radius invalid"));
		return false;
	}
	if (!FMath::IsFinite(GeneratorData->MinInitialOrbitalPhaseDegrees) ||
		!FMath::IsFinite(GeneratorData->MaxInitialOrbitalPhaseDegrees) ||
		GeneratorData->MaxInitialOrbitalPhaseDegrees < GeneratorData->MinInitialOrbitalPhaseDegrees)
	{
		LogGeneration(ELogVerbosity::Error, TEXT("Orbit phase invalid"));
		return false;
	}
	return true;
}

ASpaceNavCentralBody* ASpaceNavSystemGenerator::SpawnCentralBody()
{
	const FTransform spawnTransform = GetActorTransform();
	ASpaceNavCentralBody* centralBody = GetWorld()->SpawnActorDeferred<ASpaceNavCentralBody>(
		GeneratorData->CentralBodyClass.Get(), spawnTransform, nullptr, nullptr,
		ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
	if (centralBody == nullptr) return nullptr;

	centralBody->InitializeBody(GeneratorData->CentralBodyMassTonnes, GeneratorData->CentralBodyRadiusMeters);
	centralBody->FinishSpawning(spawnTransform);
	return centralBody;
}

int32 ASpaceNavSystemGenerator::SpawnPlanets()
{
	FRandomStream randomStream(GeneratorData->RandomSeed);
	const TArray<double> elevations = GeneratePlanetElevations(randomStream);
	bool bWarnedMissingMaterial = false;
	for (int32 planetIndex = 0; planetIndex < GeneratorData->NumberOfPlanets; ++planetIndex)
	{
		if (!SpawnPlanet(randomStream, elevations[planetIndex], bWarnedMissingMaterial)) return planetIndex;
	}
	return GeneratorData->NumberOfPlanets;
}

bool ASpaceNavSystemGenerator::SpawnMarkers()
{
	FRandomStream randomStream(GeneratorData->RandomSeed);
	const FVector radialDirection = randomStream.VRand();
	const FVector radialOffset = radialDirection * (GeneratorData->SystemBoundaryKm * UnrealUnitsPerKm);
	const FQuat spawnRotation = GetActorQuat();
	const FTransform targetTransform(spawnRotation, GetActorLocation() + radialOffset);
	const FTransform startTransform(spawnRotation, GetActorLocation() - radialOffset);
	UClass* targetClass = GeneratorData->TargetMarkerClass.Get();
	if (targetClass == nullptr) targetClass = ASpaceNavTargetMarker::StaticClass();
	UClass* startClass = GeneratorData->StartMarkerClass.Get();
	if (startClass == nullptr) startClass = ASpaceNavStartMarker::StaticClass();

	ASpaceNavTargetMarker* targetMarker = GetWorld()->SpawnActorDeferred<ASpaceNavTargetMarker>(
		targetClass, targetTransform, nullptr, nullptr, ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
	if (targetMarker == nullptr) return false;
	targetMarker->FinishSpawning(targetTransform);

	ASpaceNavStartMarker* startMarker = GetWorld()->SpawnActorDeferred<ASpaceNavStartMarker>(
		startClass, startTransform, nullptr, nullptr, ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
	if (startMarker == nullptr) return false;
	startMarker->FinishSpawning(startTransform);
	return true;
}

TArray<double> ASpaceNavSystemGenerator::GeneratePlanetElevations(FRandomStream& RandomStream) const
{
	const int32 planetCount = GeneratorData->NumberOfPlanets;
	TArray<double> elevations;
	elevations.Reserve(planetCount);
	for (int32 planetIndex = 0; planetIndex < planetCount; ++planetIndex)
	{
		elevations.Add(SampleRange(RandomStream, -MaxOrbitalElevationDegrees, MaxOrbitalElevationDegrees));
	}
	return elevations;
}

bool ASpaceNavSystemGenerator::SpawnPlanet(FRandomStream& RandomStream, double ElevationDegrees,
	bool& bWarnedMissingMaterial)
{
	const double massTonnes = SampleRange(RandomStream, GeneratorData->MinPlanetMassTonnes, GeneratorData->MaxPlanetMassTonnes);
	const double radiusMeters = SampleRange(RandomStream, GeneratorData->MinPlanetRadiusMeters, GeneratorData->MaxPlanetRadiusMeters);
	const double orbitalRadiusKm = SampleRange(RandomStream, GeneratorData->MinOrbitalRadiusKm, GeneratorData->MaxOrbitalRadiusKm);
	const double phaseDegrees = SampleRange(RandomStream, GeneratorData->MinInitialOrbitalPhaseDegrees,
		GeneratorData->MaxInitialOrbitalPhaseDegrees);
	const double phaseRadians = FMath::DegreesToRadians(phaseDegrees);
	const double elevationRadians = FMath::DegreesToRadians(ElevationDegrees);
	const double phaseCosine = FMath::Cos(phaseRadians);
	const double phaseSine = FMath::Sin(phaseRadians);
	const double elevationCosine = FMath::Cos(elevationRadians);
	const FVector localRadius(phaseCosine * elevationCosine, phaseSine * elevationCosine, FMath::Sin(elevationRadians));
	const FQuat orbitRotation = GetActorQuat();
	const FVector planetLocation = GetActorLocation() + orbitRotation.RotateVector(localRadius * (orbitalRadiusKm * UnrealUnitsPerKm));
	const double orbitalSpeed = FMath::Sqrt(GeneratorData->GravityCoefficient * GeneratorData->CentralBodyMassTonnes /
		(orbitalRadiusKm * MetersPerKm));
	const FVector localTangent(-phaseSine, phaseCosine, 0.0);
	const FVector orbitalVelocity = orbitRotation.RotateVector(localTangent * orbitalSpeed);
	UMaterialInterface* selectedMaterial = SelectPlanetMaterial(RandomStream, bWarnedMissingMaterial);
	const FTransform spawnTransform(orbitRotation, planetLocation);
	ASpaceNavPlanet* planet = GetWorld()->SpawnActorDeferred<ASpaceNavPlanet>(
		GeneratorData->PlanetClass.Get(), spawnTransform, nullptr, nullptr,
		ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
	if (planet == nullptr) return false;

	planet->InitializePlanet(massTonnes, radiusMeters, orbitalRadiusKm, phaseDegrees, orbitalVelocity, selectedMaterial);
	planet->FinishSpawning(spawnTransform);
	return true;
}

UMaterialInterface* ASpaceNavSystemGenerator::SelectPlanetMaterial(FRandomStream& RandomStream,
	bool& bWarnedMissingMaterial) const
{
	UMaterialInterface* selectedMaterial = nullptr;
	if (!GeneratorData->PlanetMaterials.IsEmpty())
	{
		const int32 materialIndex = RandomStream.RandRange(0, GeneratorData->PlanetMaterials.Num() - 1);
		selectedMaterial = GeneratorData->PlanetMaterials[materialIndex].Get();
	}
	if (selectedMaterial == nullptr && !bWarnedMissingMaterial)
	{
		LogGeneration(ELogVerbosity::Warning, TEXT("Material missing"));
		bWarnedMissingMaterial = true;
	}
	return selectedMaterial;
}

void ASpaceNavSystemGenerator::LogGeneration(ELogVerbosity::Type Verbosity, const TCHAR* Message) const
{
	if (Verbosity == ELogVerbosity::Display && !bLogGeneration)
	{
		return;
	}

	switch (Verbosity)
	{
	case ELogVerbosity::Display:
		UE_LOG(LogSpaceNavSystemGenerator, Display, TEXT("%s"), Message);
		break;
	case ELogVerbosity::Warning:
		UE_LOG(LogSpaceNavSystemGenerator, Warning, TEXT("%s"), Message);
		break;
	case ELogVerbosity::Error:
		UE_LOG(LogSpaceNavSystemGenerator, Error, TEXT("%s"), Message);
		break;
	default:
		break;
	}
}

namespace
{
	bool IsValidPositiveRange(double Minimum, double Maximum)
	{
		return FMath::IsFinite(Minimum) && FMath::IsFinite(Maximum) && Minimum > 0.0 && Maximum >= Minimum;
	}

	double SampleRange(FRandomStream& RandomStream, double Minimum, double Maximum)
	{
		return FMath::Lerp(Minimum, Maximum, static_cast<double>(RandomStream.GetFraction()));
	}
}
