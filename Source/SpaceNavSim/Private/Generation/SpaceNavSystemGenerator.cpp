#include "Generation/SpaceNavSystemGenerator.h"

#include "Generation/SpaceNavSystemGeneratorDataAsset.h"
#include "InteractiveObjects/Actors/SpaceNavCentralBody.h"
#include "InteractiveObjects/Actors/SpaceNavPlanet.h"
#include "InteractiveObjects/Actors/SpaceNavStartMarker.h"
#include "InteractiveObjects/Actors/SpaceNavTargetMarker.h"

#include "Containers/Set.h"
#include "Engine/World.h"
#include "Math/RandomStream.h"
#include "Misc/ConfigCacheIni.h"

DEFINE_LOG_CATEGORY_STATIC(LogSpaceNavSystemGenerator, Log, All);

namespace
{
	constexpr double UnrealUnitsPerKm = 100000.0;
	constexpr double UnrealUnitsPerMeter = 100.0;
	constexpr double MetersPerKm = 1000.0;
	constexpr double OrbitJitterFraction = 0.05;
	constexpr int32 RandomSlotAttemptsPerPlanet = 20;

	bool IsValidPositiveRange(double Minimum, double Maximum);
	double SampleRange(FRandomStream& RandomStream, double Minimum, double Maximum);
	bool IsPhaseAllowed(double PhaseDegrees, double MinimumDegrees, double MaximumDegrees);
	bool IsOrbitSlotAllowed(const FVector& OrbitSlotKm, double MinimumRadiusKm, double MaximumRadiusKm,
		double MinimumPhaseDegrees, double MaximumPhaseDegrees);
}

void ASpaceNavSystemGenerator::BeginPlay()
{
	Super::BeginPlay();
	if (!PrepareGeneration()) return;
	GenerateSystem();
}

ASpaceNavStartMarker* ASpaceNavSystemGenerator::GetStartMarker() const
{
	return StartMarker.Get();
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
	LogGeneration(ELogVerbosity::Display, *FString::Printf(TEXT("Orbits %d"), spawnedPlanets));
	if (!SpawnMarkers())
	{
		LogGeneration(ELogVerbosity::Error, TEXT("Failed: markers"));
		return;
	}
	LogGeneration(ELogVerbosity::Display, TEXT("Markers 2/2"));
	LogGeneration(ELogVerbosity::Display, TEXT("Done"));
	OnStartZoneReady.Broadcast();
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
	if (!FMath::IsFinite(GeneratorData->PlanetSpacingMultiplier) || GeneratorData->PlanetSpacingMultiplier <= 1.0)
	{
		LogGeneration(ELogVerbosity::Error, TEXT("Planet spacing invalid"));
		return false;
	}
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
	if (!IsValidPositiveRange(GeneratorData->MinPlanetOrbitalSpeedMetersPerSecond,
		GeneratorData->MaxPlanetOrbitalSpeedMetersPerSecond))
	{
		LogGeneration(ELogVerbosity::Error, TEXT("Orbit speed invalid"));
		return false;
	}
	if (!FMath::IsFinite(GeneratorData->MinInitialOrbitalPhaseDegrees) ||
		!FMath::IsFinite(GeneratorData->MaxInitialOrbitalPhaseDegrees) ||
		GeneratorData->MaxInitialOrbitalPhaseDegrees < GeneratorData->MinInitialOrbitalPhaseDegrees)
	{
		LogGeneration(ELogVerbosity::Error, TEXT("Orbit phase invalid"));
		return false;
	}
	const int32 availableSlots = GenerateOrbitSlots().Num();
	if (availableSlots < GeneratorData->NumberOfPlanets)
	{
		LogGeneration(ELogVerbosity::Error,
			*FString::Printf(TEXT("Failed: slots %d/%d"), availableSlots, GeneratorData->NumberOfPlanets));
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
	if (GeneratorData->NumberOfPlanets == 0) return 0;

	FRandomStream randomStream(GeneratorData->RandomSeed);
	const int32 orbitDirectionSign = randomStream.RandRange(0, 1) == 0 ? -1 : 1;
	const double outerOrbitSpeed = SampleRange(randomStream, GeneratorData->MinPlanetOrbitalSpeedMetersPerSecond,
		GeneratorData->MaxPlanetOrbitalSpeedMetersPerSecond);
	const double angularSpeedRadiansPerSecond = orbitDirectionSign * outerOrbitSpeed /
		(GeneratorData->MaxOrbitalRadiusKm * MetersPerKm);
	const TArray<FVector> orbitSlots = GenerateOrbitSlots();
	const double minimumSeparationKm = 2.0 * GeneratorData->MaxPlanetRadiusMeters *
		GeneratorData->PlanetSpacingMultiplier / MetersPerKm;
	const double jitterKm = minimumSeparationKm * OrbitJitterFraction;
	bool bWarnedMissingMaterial = false;
	for (int32 planetIndex = 0; planetIndex < GeneratorData->NumberOfPlanets; ++planetIndex)
	{
		const FVector& orbitSlot = orbitSlots[planetIndex];
		const double orbitRadiusKm = orbitSlot.Size();
		const double radialJitterKm = SampleRange(randomStream, -jitterKm, jitterKm);
		const FVector orbitPositionKm = orbitSlot * ((orbitRadiusKm + radialJitterKm) / orbitRadiusKm);
		if (!SpawnPlanet(randomStream, orbitPositionKm, angularSpeedRadiansPerSecond,
			bWarnedMissingMaterial)) return planetIndex;
	}
	return GeneratorData->NumberOfPlanets;
}

bool ASpaceNavSystemGenerator::SpawnMarkers()
{
	FRandomStream randomStream(GeneratorData->RandomSeed);
	const FVector radialDirection = randomStream.VRand();
	const FVector radialOffset = radialDirection * (GeneratorData->SystemBoundaryKm * UnrealUnitsPerKm);
	const double startDistance = SampleRange(randomStream, 2.0, 5.0) *
		GeneratorData->CentralBodyRadiusMeters * UnrealUnitsPerMeter;
	const FQuat spawnRotation = GetActorQuat();
	const FTransform targetTransform(spawnRotation, GetActorLocation() + radialOffset);
	const FTransform startTransform(spawnRotation, GetActorLocation() - radialDirection * startDistance);
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
	StartMarker = startMarker;
	return true;
}

TArray<FVector> ASpaceNavSystemGenerator::GenerateOrbitSlots() const
{
	const double minimumSeparationKm = 2.0 * GeneratorData->MaxPlanetRadiusMeters *
		GeneratorData->PlanetSpacingMultiplier / MetersPerKm;
	const double jitterKm = minimumSeparationKm * OrbitJitterFraction;
	const double latticeSpacingKm = minimumSeparationKm + 2.0 * jitterKm;
	const double minimumRadiusKm = GeneratorData->MinOrbitalRadiusKm + jitterKm;
	const double maximumRadiusKm = GeneratorData->MaxOrbitalRadiusKm - jitterKm;
	TArray<FVector> orbitSlots;
	if (minimumRadiusKm > maximumRadiusKm) return orbitSlots;

	const int32 maximumIndex = FMath::FloorToInt(maximumRadiusKm / latticeSpacingKm);
	FRandomStream randomStream(GeneratorData->RandomSeed);
	TSet<FIntVector> examinedSlots;
	auto tryAddSlot = [&](const FIntVector& slotIndex)
	{
		if (examinedSlots.Contains(slotIndex)) return;
		examinedSlots.Add(slotIndex);
		const FVector orbitSlotKm(slotIndex.X * latticeSpacingKm, slotIndex.Y * latticeSpacingKm,
			slotIndex.Z * latticeSpacingKm);
		if (!IsOrbitSlotAllowed(orbitSlotKm, minimumRadiusKm, maximumRadiusKm,
			GeneratorData->MinInitialOrbitalPhaseDegrees, GeneratorData->MaxInitialOrbitalPhaseDegrees)) return;
		orbitSlots.Add(orbitSlotKm);
	};

	const int64 randomAttemptCount = static_cast<int64>(GeneratorData->NumberOfPlanets) * RandomSlotAttemptsPerPlanet;
	for (int64 attemptIndex = 0; attemptIndex < randomAttemptCount &&
		orbitSlots.Num() < GeneratorData->NumberOfPlanets; ++attemptIndex)
	{
		tryAddSlot(FIntVector(randomStream.RandRange(-maximumIndex, maximumIndex),
			randomStream.RandRange(-maximumIndex, maximumIndex),
			randomStream.RandRange(-maximumIndex, maximumIndex)));
	}
	for (int32 xIndex = -maximumIndex; xIndex <= maximumIndex &&
		orbitSlots.Num() < GeneratorData->NumberOfPlanets; ++xIndex)
	{
		for (int32 yIndex = -maximumIndex; yIndex <= maximumIndex &&
			orbitSlots.Num() < GeneratorData->NumberOfPlanets; ++yIndex)
		{
			for (int32 zIndex = -maximumIndex; zIndex <= maximumIndex &&
				orbitSlots.Num() < GeneratorData->NumberOfPlanets; ++zIndex)
			{
				tryAddSlot(FIntVector(xIndex, yIndex, zIndex));
			}
		}
	}
	return orbitSlots;
}

bool ASpaceNavSystemGenerator::SpawnPlanet(FRandomStream& RandomStream, const FVector& LocalOrbitPositionKm,
	double SignedAngularSpeedRadiansPerSecond, bool& bWarnedMissingMaterial)
{
	const double massTonnes = SampleRange(RandomStream, GeneratorData->MinPlanetMassTonnes, GeneratorData->MaxPlanetMassTonnes);
	const double radiusMeters = SampleRange(RandomStream, GeneratorData->MinPlanetRadiusMeters, GeneratorData->MaxPlanetRadiusMeters);
	const double orbitalRadiusKm = LocalOrbitPositionKm.Size();
	const double rawPhaseDegrees = FMath::RadiansToDegrees(FMath::Atan2(LocalOrbitPositionKm.Y, LocalOrbitPositionKm.X));
	const double minimumPhaseDegrees = GeneratorData->MinInitialOrbitalPhaseDegrees;
	const double phaseDegrees = minimumPhaseDegrees + FMath::Fmod(
		FMath::Fmod(rawPhaseDegrees - minimumPhaseDegrees, 360.0) + 360.0, 360.0);
	const FQuat orbitRotation = GetActorQuat();
	const FVector localPositionUU = LocalOrbitPositionKm * UnrealUnitsPerKm;
	const FVector planetLocation = GetActorLocation() + orbitRotation.RotateVector(localPositionUU);
	const FVector localOrbitalVelocity(-LocalOrbitPositionKm.Y * SignedAngularSpeedRadiansPerSecond * MetersPerKm,
		LocalOrbitPositionKm.X * SignedAngularSpeedRadiansPerSecond * MetersPerKm, 0.0);
	const FVector orbitalVelocity = orbitRotation.RotateVector(localOrbitalVelocity);
	UMaterialInterface* selectedMaterial = SelectPlanetMaterial(RandomStream, bWarnedMissingMaterial);
	const FTransform spawnTransform(orbitRotation, planetLocation);
	ASpaceNavPlanet* planet = GetWorld()->SpawnActorDeferred<ASpaceNavPlanet>(
		GeneratorData->PlanetClass.Get(), spawnTransform, nullptr, nullptr,
		ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
	if (planet == nullptr) return false;

	planet->FinishSpawning(spawnTransform);
	planet->InitializePlanet(massTonnes, radiusMeters, orbitalRadiusKm, phaseDegrees, GetActorLocation(),
		GetActorUpVector(), SignedAngularSpeedRadiansPerSecond, orbitalVelocity, selectedMaterial);
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

	bool IsPhaseAllowed(double PhaseDegrees, double MinimumDegrees, double MaximumDegrees)
	{
		if (MaximumDegrees - MinimumDegrees >= 360.0) return true;
		double relativePhaseDegrees = FMath::Fmod(PhaseDegrees - MinimumDegrees, 360.0);
		if (relativePhaseDegrees < 0.0) relativePhaseDegrees += 360.0;
		return relativePhaseDegrees <= MaximumDegrees - MinimumDegrees;
	}

	bool IsOrbitSlotAllowed(const FVector& OrbitSlotKm, double MinimumRadiusKm, double MaximumRadiusKm,
		double MinimumPhaseDegrees, double MaximumPhaseDegrees)
	{
		const double orbitalRadiusKm = OrbitSlotKm.Size();
		if (orbitalRadiusKm < MinimumRadiusKm || orbitalRadiusKm > MaximumRadiusKm) return false;
		if (OrbitSlotKm.X == 0.0 && OrbitSlotKm.Y == 0.0) return false;
		const double phaseDegrees = FMath::RadiansToDegrees(FMath::Atan2(OrbitSlotKm.Y, OrbitSlotKm.X));
		return IsPhaseAllowed(phaseDegrees, MinimumPhaseDegrees, MaximumPhaseDegrees);
	}
}
