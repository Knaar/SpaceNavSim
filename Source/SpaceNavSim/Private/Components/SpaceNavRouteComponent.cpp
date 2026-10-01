#include "Components/SpaceNavRouteComponent.h"

#include "DrawDebugHelpers.h"
#include "EngineUtils.h"
#include "Game/Pawn/SpaceNavPawnSettingsDataAsset.h"
#include "InteractiveObjects/Actors/SpaceNavCentralBody.h"
#include "InteractiveObjects/Actors/SpaceNavPlanet.h"
#include "InteractiveObjects/Actors/SpaceNavTargetMarker.h"

DEFINE_LOG_CATEGORY_STATIC(LogSpaceNavRoute, Log, All);

namespace
{
	constexpr double UnrealUnitsPerMeter = 100.0;
	constexpr double WaypointRadiusScale = 2.0;
	constexpr int32 CornerSampleCount = 12;
}

void USpaceNavRouteComponent::BuildRoute()
{
	LogDisplay(TEXT("Started"));
	if (!CheckSettings()) return;
	FVector targetLocation;
	if (!FindTargetLocation(targetLocation)) return;

	TArray<FSphere> obstacles;
	if (!CollectObstacles(obstacles)) return;
	if (!ComputeRoute(targetLocation, obstacles)) return;
	if (!SmoothRoute(obstacles)) return;
	DrawRoute();
}

bool USpaceNavRouteComponent::CheckSettings() const
{
	if (PawnSettings == nullptr)
	{
		UE_LOG(LogSpaceNavRoute, Error, TEXT("Failed: no settings"));
		return false;
	}
	LogDisplay(TEXT("Settings ready"));
	return true;
}

bool USpaceNavRouteComponent::FindTargetLocation(FVector& OutTargetLocation) const
{
	for (TActorIterator<ASpaceNavTargetMarker> targetIterator(GetWorld()); targetIterator; ++targetIterator)
	{
		OutTargetLocation = targetIterator->GetActorLocation();
		LogDisplay(TEXT("Target found"));
		return true;
	}

	UE_LOG(LogSpaceNavRoute, Error, TEXT("Failed: target"));
	return false;
}

bool USpaceNavRouteComponent::CollectObstacles(TArray<FSphere>& OutObstacles) const
{
	const FVector startLocation = GetOwner()->GetActorLocation();
	int32 skippedStartBodies = 0;

	for (TActorIterator<ASpaceNavCentralBody> bodyIterator(GetWorld()); bodyIterator; ++bodyIterator)
	{
		const double radius = bodyIterator->RadiusMeters * UnrealUnitsPerMeter;
		if (!FMath::IsFinite(radius) || radius <= 0.0) continue;
		const FVector center = bodyIterator->GetActorLocation();
		if (FVector::DistSquared(startLocation, center) < radius * radius)
		{
			++skippedStartBodies;
			continue;
		}
		OutObstacles.Emplace(center, radius);
	}

	for (TActorIterator<ASpaceNavPlanet> planetIterator(GetWorld()); planetIterator; ++planetIterator)
	{
		const double radius = planetIterator->RadiusMeters *
			(1.0 + PawnSettings->PlanetSafetyBufferFraction) * UnrealUnitsPerMeter;
		if (!FMath::IsFinite(radius) || radius <= 0.0) continue;
		const FVector center = planetIterator->GetActorLocation();
		if (FVector::DistSquared(startLocation, center) < radius * radius)
		{
			UE_LOG(LogSpaceNavRoute, Error, TEXT("Failed: start zone"));
			return false;
		}
		OutObstacles.Emplace(center, radius);
	}

	if (skippedStartBodies > 0)
	{
		UE_LOG(LogSpaceNavRoute, Warning, TEXT("Start in body %d"), skippedStartBodies);
	}
	if (bLogRoute)
	{
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Bodies %d"), OutObstacles.Num());
	}
	return true;
}

bool USpaceNavRouteComponent::ComputeRoute(const FVector& TargetLocation, const TArray<FSphere>& Obstacles)
{
	TArray<FVector> nodes;
	CreateNodes(TargetLocation, Obstacles, nodes);

	TArray<int32> previous;
	if (!FindShortestPath(nodes, Obstacles, previous))
	{
		UE_LOG(LogSpaceNavRoute, Error, TEXT("Failed: route"));
		return false;
	}

	TArray<int32> reversePath;
	for (int32 nodeIndex = 1; nodeIndex != INDEX_NONE; nodeIndex = previous[nodeIndex])
	{
		reversePath.Add(nodeIndex);
	}
	RoutePoints.Reset(reversePath.Num());
	for (int32 pathIndex = reversePath.Num() - 1; pathIndex >= 0; --pathIndex)
	{
		RoutePoints.Add(nodes[reversePath[pathIndex]]);
	}
	if (bLogRoute)
	{
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Route %d pts"), RoutePoints.Num());
	}
	return true;
}

bool USpaceNavRouteComponent::SmoothRoute(const TArray<FSphere>& Obstacles)
{
	const TArray<FVector> corners = RoutePoints;
	TArray<FVector> smoothedPoints;
	double blendFraction = 0.25;
	for (int32 attempt = 0; attempt < 10; ++attempt)
	{
		BuildSmoothedPoints(corners, blendFraction, smoothedPoints);
		if (IsRouteClear(smoothedPoints, Obstacles))
		{
			RoutePoints = MoveTemp(smoothedPoints);
			if (bLogRoute)
			{
				UE_LOG(LogSpaceNavRoute, Display, TEXT("Smooth %d pts"), RoutePoints.Num());
			}
			return true;
		}
		blendFraction *= 0.5;
	}

	UE_LOG(LogSpaceNavRoute, Error, TEXT("Failed: smooth"));
	return false;
}

void USpaceNavRouteComponent::DrawRoute() const
{
	for (int32 pointIndex = 1; pointIndex < RoutePoints.Num(); ++pointIndex)
	{
		DrawDebugLine(GetWorld(), RoutePoints[pointIndex - 1], RoutePoints[pointIndex],
			FColor::Cyan, true, -1.0f, 0, 30.0f);
	}
	LogDisplay(TEXT("Done"));
}

void USpaceNavRouteComponent::CreateNodes(const FVector& TargetLocation,
	const TArray<FSphere>& Obstacles, TArray<FVector>& OutNodes) const
{
	OutNodes.Reserve(2 + Obstacles.Num() * 6);
	OutNodes.Add(GetOwner()->GetActorLocation());
	OutNodes.Add(TargetLocation);

	const FVector directions[] = {
		FVector::ForwardVector, -FVector::ForwardVector,
		FVector::RightVector, -FVector::RightVector,
		FVector::UpVector, -FVector::UpVector
	};
	for (const FSphere& obstacle : Obstacles)
	{
		for (const FVector& direction : directions)
		{
			OutNodes.Add(obstacle.Center + direction * obstacle.W * WaypointRadiusScale);
		}
	}
}

bool USpaceNavRouteComponent::FindShortestPath(const TArray<FVector>& Nodes,
	const TArray<FSphere>& Obstacles, TArray<int32>& OutPrevious) const
{
	const int32 nodeCount = Nodes.Num();
	TArray<double> distances;
	distances.Init(TNumericLimits<double>::Max(), nodeCount);
	OutPrevious.Init(INDEX_NONE, nodeCount);
	TArray<uint8> visited;
	visited.Init(0, nodeCount);
	distances[0] = 0.0;

	for (int32 iteration = 0; iteration < nodeCount; ++iteration)
	{
		int32 currentIndex = INDEX_NONE;
		for (int32 nodeIndex = 0; nodeIndex < nodeCount; ++nodeIndex)
		{
			if (!visited[nodeIndex] &&
				(currentIndex == INDEX_NONE || distances[nodeIndex] < distances[currentIndex]))
			{
				currentIndex = nodeIndex;
			}
		}
		if (currentIndex == INDEX_NONE || distances[currentIndex] == TNumericLimits<double>::Max()) break;
		if (currentIndex == 1) return true;
		visited[currentIndex] = 1;

		for (int32 neighborIndex = 0; neighborIndex < nodeCount; ++neighborIndex)
		{
			if (visited[neighborIndex] || neighborIndex == currentIndex ||
				!IsSegmentClear(Nodes[currentIndex], Nodes[neighborIndex], Obstacles)) continue;
			const double candidateDistance = distances[currentIndex] +
				FVector::Dist(Nodes[currentIndex], Nodes[neighborIndex]);
			if (candidateDistance >= distances[neighborIndex]) continue;
			distances[neighborIndex] = candidateDistance;
			OutPrevious[neighborIndex] = currentIndex;
		}
	}
	return false;
}

void USpaceNavRouteComponent::BuildSmoothedPoints(const TArray<FVector>& Corners,
	double BlendFraction, TArray<FVector>& OutPoints) const
{
	OutPoints.Reset();
	if (Corners.IsEmpty()) return;
	OutPoints.Add(Corners[0]);

	for (int32 cornerIndex = 1; cornerIndex < Corners.Num() - 1; ++cornerIndex)
	{
		const FVector toPrevious = Corners[cornerIndex - 1] - Corners[cornerIndex];
		const FVector toNext = Corners[cornerIndex + 1] - Corners[cornerIndex];
		const double cutDistance = FMath::Min(toPrevious.Size(), toNext.Size()) * BlendFraction;
		if (cutDistance <= 0.0)
		{
			OutPoints.Add(Corners[cornerIndex]);
			continue;
		}

		const FVector entry = Corners[cornerIndex] + toPrevious.GetSafeNormal() * cutDistance;
		const FVector exit = Corners[cornerIndex] + toNext.GetSafeNormal() * cutDistance;
		OutPoints.Add(entry);
		for (int32 sampleIndex = 1; sampleIndex <= CornerSampleCount; ++sampleIndex)
		{
			const double fraction = static_cast<double>(sampleIndex) / CornerSampleCount;
			const double remaining = 1.0 - fraction;
			OutPoints.Add(entry * (remaining * remaining) +
				Corners[cornerIndex] * (2.0 * remaining * fraction) + exit * (fraction * fraction));
		}
	}
	if (Corners.Num() > 1) OutPoints.Add(Corners.Last());
}

bool USpaceNavRouteComponent::IsRouteClear(const TArray<FVector>& Points,
	const TArray<FSphere>& Obstacles) const
{
	if (Points.Num() < 2) return false;
	for (int32 pointIndex = 1; pointIndex < Points.Num(); ++pointIndex)
	{
		if (!IsSegmentClear(Points[pointIndex - 1], Points[pointIndex], Obstacles)) return false;
	}
	return true;
}

bool USpaceNavRouteComponent::IsSegmentClear(const FVector& Start, const FVector& End,
	const TArray<FSphere>& Obstacles) const
{
	const FVector segment = End - Start;
	const double segmentLengthSquared = segment.SizeSquared();
	for (const FSphere& obstacle : Obstacles)
	{
		const double fraction = segmentLengthSquared > 0.0
			? FMath::Clamp(FVector::DotProduct(obstacle.Center - Start, segment) / segmentLengthSquared, 0.0, 1.0)
			: 0.0;
		const FVector closestPoint = Start + segment * fraction;
		if (FVector::DistSquared(closestPoint, obstacle.Center) < obstacle.W * obstacle.W) return false;
	}
	return true;
}

void USpaceNavRouteComponent::LogDisplay(const TCHAR* Message) const
{
	if (!bLogRoute) return;
	UE_LOG(LogSpaceNavRoute, Display, TEXT("%s"), Message);
}
