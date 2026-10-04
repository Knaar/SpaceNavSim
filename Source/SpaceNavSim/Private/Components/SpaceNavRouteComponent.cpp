#include "Components/SpaceNavRouteComponent.h"

#include <cfloat>

#include "Algo/Reverse.h"
#include "Async/Async.h"
#include "Components/LineBatchComponent.h"
#include "Components/SpaceNavEngineComponent.h"
#include "Components/SpaceNavResourceStoreComponent.h"
#include "EngineUtils.h"
#include "Game/Pawn/SpaceNavPawn.h"
#include "Game/Pawn/SpaceNavPawnSettingsDataAsset.h"
#include "Generation/SpaceNavSystemGenerator.h"
#include "Generation/SpaceNavSystemGeneratorDataAsset.h"
#include "InteractiveObjects/Actors/SpaceNavCentralBody.h"
#include "InteractiveObjects/Actors/SpaceNavPlanet.h"
#include "InteractiveObjects/Actors/SpaceNavTargetMarker.h"
#include "TimerManager.h"
#include "HAL/PlatformTime.h"

DEFINE_LOG_CATEGORY_STATIC(LogSpaceNavRoute, Log, All);

namespace
{
	constexpr double UnrealUnitsPerMeter = 100.0;
	constexpr int32 CornerSampleCount = 12;
	constexpr double AlignmentToleranceDegrees = 5.0;
	constexpr double ArrivalDistance = 200.0;
	constexpr double SteeringLookAheadSeconds = 0.5;
	constexpr double LiveThreatLookAheadSeconds = 2.0;
	constexpr double ReferenceSampleIntervalSeconds = 0.25;
	constexpr double MinimumBodyReserveFraction = 0.1;
	constexpr double CorrectionWindowSeconds = 2.0;
	constexpr double MaxForwardFuelPerSecond = 1.0;
	constexpr double MaxLateralFuelPerSecond = 5.0;
	constexpr double MaxTurnFuelPerSecond = 5.0;
	constexpr int32 MaximumPredictionSteps = 20000;
	

	enum class EPlanFamily : uint8
	{
		SparseBurns,
		ContinuousCorrection
	};

	enum class EGuideStyle : uint8
	{
		Fuel,
		Time,
		Balanced
	};

	struct FHierarchicalGuide
	{
		TArray<FVector> Points;
		EGuideStyle Style = EGuideStyle::Balanced;
		int32 BendCount = 0;
		double EstimatedFuelKg = 0.0;
		double EstimatedTimeSeconds = 0.0;
		double EstimatedRiskCost = 0.0;
	};
	struct FEngineSnapshot
	{
		float ThrustAcceleration = 0.0f;
		float MaxSpeed = 0.0f;
		float TurnImpulsePerFuelUnit = 0.0f;
		float TurnDeceleration = 0.0f;
		float MaxTurnSpeed = 0.0f;
	};

	FEngineSnapshot SnapshotEngine(const USpaceNavEngineComponent& Engine);

	struct FNavigationState
	{
		FVector Position = FVector::ZeroVector;
		FQuat Orientation = FQuat::Identity;
		FVector EngineVelocity = FVector::ZeroVector;
		FVector GravityVelocity = FVector::ZeroVector;
		float YawAngularVelocity = 0.0f;
		float PitchAngularVelocity = 0.0f;
		double FuelKg = 0.0;
		double ElapsedSeconds = 0.0;
		int32 NextPointIndex = 1;
	};

	struct FNavigationControl
	{
		float ForwardFuel = 0.0f;
		float RightFuel = 0.0f;
		float UpFuel = 0.0f;
		float YawFuel = 0.0f;
		float PitchFuel = 0.0f;
	};

	struct FPlannerBody
	{
		TWeakObjectPtr<AActor> Actor;
		bool bPlanet = false;
		ASpaceNavPlanet::FOrbitSnapshot Orbit;
		FVector FixedCenter = FVector::ZeroVector;
		double MassTonnes = 0.0;
		double RadiusUU = 0.0;
		double AvoidRadiusUU = 0.0;
		double CriticalRadiusUU = 0.0;

		FVector LocationAfter(double SecondsAhead) const;
	};

	struct FPlannerContext
	{
		FEngineSnapshot Engine;
		FNavigationState InitialState;
		TArray<FPlannerBody> Bodies;
		TArray<FSphere> Obstacles;
		TArray<FVector> EscapePoints;
		FVector Target = FVector::ZeroVector;
		double GravityCoefficient = 0.0;
		double InitialFuelKg = 0.0;

		int32 FuelMaxTurns = 2;
		TArray<double> CruiseSpeedFractions;
		int32 MaxGuideCandidates = 200;
		int32 MaxDetailedCandidates = 32;
		double MinimumRouteSeparationFraction = 0.02;
		double CriticalBodyClearanceRadiusFraction = 0.6;
		double ShipToPlanetSpeedFactor = 2.0;
		double DesiredCorrectionFuelReserveFraction = 0.2;
		double ManeuverVelocityErrorFraction = 0.01;
		FSpaceNavRouteWeightSet ModeWeights[3];

	};

	struct FScriptedGuardScratch
	{
		TArray<int32> BodyIndices;
		TArray<FVector> PreviousBodies;
		TArray<FVector> CurrentBodies;
	};
	struct FPredictedStep
	{
		double TimeSeconds = 0.0;
		FVector Position = FVector::ZeroVector;
		FNavigationControl Control;
	};

	struct FPredictedStateSample
	{
		double TimeSeconds = 0.0;
		FVector Position = FVector::ZeroVector;
		FVector Velocity = FVector::ZeroVector;
	};

	enum class EFlightPhase : uint8 { Align, Launch, Coast, Brake, Redirect };

	struct FTimeCurveGeometry
	{
		FVector StartPosition = FVector::ZeroVector;
		FVector Control1 = FVector::ZeroVector;
		FVector Control2 = FVector::ZeroVector;
		FVector EndPosition = FVector::ZeroVector;
		TArray<double> ArcLengths;
	};

	struct FScriptedMotionSegment
	{
		double StartSeconds = 0.0;
		double DurationSeconds = 0.0;
		FVector StartPosition = FVector::ZeroVector;
		FVector EndPosition = FVector::ZeroVector;
		FQuat StartOrientation = FQuat::Identity;
		FQuat EndOrientation = FQuat::Identity;
		double InitialSpeedUU = 0.0;
		double AccelerationUU = 0.0;
		double DistanceUU = 0.0;
		FNavigationControl Fuel;
		EFlightPhase Phase = EFlightPhase::Coast;
		int32 NextPointIndex = 1;
		bool bCurved = false;
		TSharedPtr<const FTimeCurveGeometry, ESPMode::ThreadSafe> Curve;
		double CurveStartDistanceUU = 0.0;
		int32 CurveFuelGroup = INDEX_NONE;
		double CurveRemainingFuelKg = 0.0;
	};

	struct FScriptedPose
	{
		FVector Position = FVector::ZeroVector;
		FVector Velocity = FVector::ZeroVector;
		FQuat Orientation = FQuat::Identity;
		EFlightPhase Phase = EFlightPhase::Coast;
		int32 NextPointIndex = 1;
	};

	FScriptedPose EvaluateTimeCurvePose(const FScriptedMotionSegment& Segment, double Seconds);
	double GetTimeCurveDeviationUU(const FScriptedMotionSegment& Segment, double FromSeconds, double ToSeconds);
	double GetScriptedFuelKg(const FNavigationControl& Fuel);
	double GetScriptedFuelToleranceKg(double InitialFuelKg);
	void AddScriptedControl(FNavigationControl& Total, const FNavigationControl& Fuel, double Fraction);
	FScriptedPose EvaluateScriptedMotion(const TArray<FScriptedMotionSegment>& Segments,
		double Seconds, int32* InOutSegmentIndex = nullptr);
	FNavigationControl AccumulateScriptedControl(const TArray<FScriptedMotionSegment>& Segments,
		double FromSeconds, double ToSeconds);

	struct FRouteCandidate
	{
		EPlanFamily Family = EPlanFamily::ContinuousCorrection;
		TArray<FVector> Guidance;
		TArray<FVector> PredictedPath;
		TArray<FScriptedMotionSegment> MotionSegments;
		TArray<FPredictedStateSample> PredictedStates;
		TArray<FPredictedStep> Maneuvers;
		double FuelUsedKg = 0.0;
		double FlightTimeSeconds = 0.0;
		double BurnDurationSeconds = 0.0;
		double CruiseSpeedFraction = 1.0;
		double CoastDurationSeconds = 0.0;
		int32 MacroTurnCount = 0;
		double ForwardFuelUsedKg = 0.0;
		double LateralFuelUsedKg = 0.0;
		double TurnFuelUsedKg = 0.0;
		int32 PredictionSteps = 0;
		double MinimumStarClearanceUU = TNumericLimits<double>::Max();
		double MinimumPlanetClearanceUU = TNumericLimits<double>::Max();
		double RiskCost = 0.0;
		double DistanceRisk = 0.0;
		double ApproachSpeedRisk = 0.0;
		double SpeedAdvantageRisk = 0.0;
		double ManeuverSensitivityRisk = 0.0;
		double CorrectionFuelRisk = 0.0;
		double MaximumManeuverDeviationUU = 0.0;
		double MinimumManeuverMarginUU = TNumericLimits<double>::Max();
		double RequiredCorrectionFuelKg = 0.0;
		double AvailableCorrectionFuelKg = 0.0;
		double DesiredCorrectionFuelKg = 0.0;
	};

	struct FSimulationFailure;
	bool BuildTimeCurveCandidate(const FRouteCandidate& Base, const FPlannerContext& Context,
		double HandleFraction, FRouteCandidate& OutCandidate, FSimulationFailure* OutFailure = nullptr);

	struct FBlockedPath
	{
		TArray<FVector> Guidance;
		int32 BodyIndex = INDEX_NONE;
		int32 NextPointIndex = INDEX_NONE;
		double TimeSeconds = 0.0;
	};

	enum class ESimulationFailureReason : uint8
	{
		InvalidInput,
		InvalidStep,
		NonFiniteState,
		StarCollision,
		PlanetCollision,
		Fuel,
		Unverified,
		Count
	};

	struct FSimulationFailure
	{
		ESimulationFailureReason Reason = ESimulationFailureReason::InvalidInput;
		double ElapsedSeconds = 0.0;
		double TargetDistanceUU = 0.0;
		double RemainingFuelKg = 0.0;
		double PhysicalGapUU = 0.0;
		double RequiredGapUU = 0.0;
		double RequiredFuelKg = 0.0;
		int32 BodyIndex = INDEX_NONE;
	};

	void RefreshRouteSafetyContext(FPlannerContext& Context);
	bool ValidateScriptedWindow(const FRouteCandidate& Candidate, double FromSeconds, double ToSeconds,
		const FPlannerContext& Context, double PhaseSlackSeconds, FScriptedGuardScratch& Scratch,
		FSimulationFailure& OutFailure, int32 InitialSegmentIndex = 0);
	void LogRouteSafetyResult(const TCHAR* Stage, const FSimulationFailure* Failure, double Milliseconds, bool bLog);
	struct FSimulationFailureSummary
	{
		int32 Counts[static_cast<int32>(ESimulationFailureReason::Count)] = {};
		FSimulationFailure Samples[static_cast<int32>(ESimulationFailureReason::Count)];

		void Add(const FSimulationFailure& Failure)
		{
			const int32 index = static_cast<int32>(Failure.Reason);
			if (Counts[index]++ == 0) Samples[index] = Failure;
		}

		void Log(const TCHAR* Stage, bool bEnabled, const FPlannerContext& Context) const
		{
			if (!bEnabled) return;
			const TCHAR* reasonNames[] = {TEXT("input"), TEXT("step"), TEXT("nonfinite"),
				TEXT("star"), TEXT("planet"), TEXT("fuel"), TEXT("unverified")};
			bool bHeaderLogged = false;
			for (int32 reasonIndex = 0; reasonIndex < static_cast<int32>(ESimulationFailureReason::Count); ++reasonIndex)
			{
				if (Counts[reasonIndex] == 0) continue;
				if (!bHeaderLogged)
				{
					UE_LOG(LogSpaceNavRoute, Display, TEXT("%s rejects"), Stage);
					bHeaderLogged = true;
				}
				const FSimulationFailure& sample = Samples[reasonIndex];
				UE_LOG(LogSpaceNavRoute, Display, TEXT("%s n%d"), reasonNames[reasonIndex], Counts[reasonIndex]);
				UE_LOG(LogSpaceNavRoute, Display, TEXT("Sample t %.2f"), sample.ElapsedSeconds);
				UE_LOG(LogSpaceNavRoute, Display, TEXT("Target m %.3g"),
					sample.TargetDistanceUU / UnrealUnitsPerMeter);
				UE_LOG(LogSpaceNavRoute, Display, TEXT("Fuel left kg %.3g"), sample.RemainingFuelKg);
				if (sample.Reason == ESimulationFailureReason::Fuel)
					UE_LOG(LogSpaceNavRoute, Display, TEXT("Need fuel %.3gkg"), sample.RequiredFuelKg);
				if (!Context.Bodies.IsValidIndex(sample.BodyIndex)) continue;
				UE_LOG(LogSpaceNavRoute, Display, TEXT("Body %d"), sample.BodyIndex);
				UE_LOG(LogSpaceNavRoute, Display, TEXT("Gap m %.3g"), sample.PhysicalGapUU / UnrealUnitsPerMeter);
				UE_LOG(LogSpaceNavRoute, Display, TEXT("Need m %.3g"), sample.RequiredGapUU / UnrealUnitsPerMeter);
			}
		}
	};

	struct FPhasedFlightState
	{
		EFlightPhase Phase = EFlightPhase::Align;
		int32 NextPointIndex = 1;
		double CruiseSpeedUU = 0.0;
		FVector PreviousGravityVelocity = FVector::ZeroVector;
		FVector SegmentDirection = FVector::ZeroVector;
		int32 SegmentPointIndex = INDEX_NONE;
		FVector PreviousPosition = FVector::ZeroVector;
		bool bHasPreviousPosition = false;
		double NextCorrectionSeconds = 0.0;
		bool bCornerBraked = false;
		int32 BrakingPointIndex = INDEX_NONE;
	};

	struct FFlightFeedback
	{
		bool bTracking = false;
		FVector TargetVelocity = FVector::ZeroVector;
		bool bAvoidance = false;
		FVector AvoidanceDirection = FVector::ZeroVector;
		double MinimumAvoidanceSpeed = 0.0;
	};

	FNavigationControl CalculatePredictedControl(const TArray<FVector>& Points,
		FNavigationState& State, const FEngineSnapshot& Engine, EPlanFamily Family,
		double CruiseSpeedFraction, float DeltaTime, FPhasedFlightState& Flight,
		const FFlightFeedback& Feedback = {});


	enum class EAutopilotActuator : uint8
	{
		YawRight, YawLeft, PitchUp, PitchDown,
		Right, Left, Up, Down, Main, Count
	};

	constexpr int32 AutopilotActuatorCount =
		static_cast<int32>(EAutopilotActuator::Count);

	struct FActuatorTelemetry
	{
		double CommandedKg = 0.0;
		double CappedKg = 0.0;
		double ObservedSpentKg = 0.0;
		double ActiveSeconds = 0.0;
		uint32 Count = 0;
	};

	struct FAutopilotTelemetry
	{
		FActuatorTelemetry Axes[AutopilotActuatorCount];
		double WindowSeconds = 0.0;
		double InitialFuelKg = 0.0;
		double LastFuelKg = 0.0;
		double FlightCommandedKg = 0.0;
		double FlightCappedKg = 0.0;
		double FlightObservedSpentKg = 0.0;
		double FuelRoundingKg = 0.0;
		double ReasonSeconds[4] = {};
		double ReasonSpentKg[4] = {};
		uint32 PhaseChanges = 0;
		uint32 ReasonChanges = 0;
		EFlightPhase WindowFirstPhase = EFlightPhase::Align;
		EFlightPhase LastPhase = EFlightPhase::Align;
		uint8 WindowFirstReason = 0;
		uint8 LastReason = 0;
		bool bHasState = false;
	};

	struct FAutopilotSnapshot
	{
		double ElapsedSeconds = 0.0;
		int32 SegmentIndex = 0;
		int32 SegmentCount = 0;
		double SpeedUUPerSecond = 0.0;
		double FuelKg = 0.0;
		double GoalDistanceUU = 0.0;
		double CrossLineErrorUU = 0.0;
		double HeadingVelocityAngleDegrees = 0.0;
		double YawRateDegreesPerSecond = 0.0;
		double PitchRateDegreesPerSecond = 0.0;
	};

	void ResetAutopilotTelemetry(FAutopilotTelemetry& Telemetry, double InitialFuelKg);
	void ObserveAutopilotTelemetry(FAutopilotTelemetry& Telemetry,
		EFlightPhase Phase, uint8 ReasonMask, float DeltaTime);
	bool BroadcastAutopilotControl(ASpaceNavPawn& Pawn,
		USpaceNavResourceStoreComponent& Resources, FAutopilotTelemetry& Telemetry,
		const FNavigationControl& Requested, const FNavigationControl& Capped, float DeltaTime,
		TFunctionRef<bool()> IsCurrentFlight);
	void FlushAutopilotTelemetry(FAutopilotTelemetry& Telemetry,
		const FAutopilotSnapshot& Snapshot, bool bLog, bool bForce = false);
	void LogAutopilotSummary(const FAutopilotTelemetry& Telemetry,
		const TCHAR* EndReason, bool bLog);



	struct FSpatialRouteReference
	{
		FVector Position = FVector::ZeroVector;
		FVector Velocity = FVector::ZeroVector;
		FVector Direction = FVector::ZeroVector;
		double TimeSeconds = 0.0;
		double CrossLineErrorUU = 0.0;
		int32 NextStatePointIndex = 1;
	};

	struct FLiveFlightThreat
	{
		FVector Direction = FVector::ZeroVector;
		double MinimumVelocityChangeUU = 0.0;
		double GapUU = 0.0;
		bool bDetected = false;
		bool bStar = false;
		bool bUnreachable = false;
	};

	FNavigationState SnapshotNavigationState(const ASpaceNavPawn& Pawn,
		const USpaceNavEngineComponent& Engine,
		const USpaceNavResourceStoreComponent& Resources, double ElapsedSeconds);
	FSpatialRouteReference FindSpatialRouteReference(const FRouteCandidate& Candidate,
		const FVector& Position, double SearchDistanceUU,
		int32& NextPathPointIndex, int32& NextStatePointIndex);
	FFlightFeedback BuildTrackingFeedback(const FRouteCandidate& Candidate,
		const FSpatialRouteReference& Reference, const FNavigationState& State,
		double TargetDistanceUU);
	template<typename MonitoredBodyType>
	FLiveFlightThreat FindLiveFlightThreat(const TArray<MonitoredBodyType>& Bodies,
		const FNavigationState& State, const FVector& GravityAcceleration,
		const FSpatialRouteReference& Reference, const FRouteCandidate& Candidate,
		const FEngineSnapshot& Engine, float DeltaTime);
	void LimitFlightControlRates(FNavigationControl& Control, float DeltaTime);
	FAutopilotSnapshot MakeAutopilotSnapshot(const FNavigationState& State,
		double FuelKg, const FVector& Target, double CrossLineErrorUU);

	bool HasReachedFlightTarget(const FVector& Position, const FVector& PreviousPosition,
		bool bHasPreviousPosition, const FVector& Target);

	void AdvanceRoutePointIndex(const TArray<FVector>& Points, const FVector& Location, int32& NextPointIndex);
	FVector ProjectOntoCurrentSegment(const TArray<FVector>& Points, const FVector& Location, int32 NextPointIndex);
	FVector FindLookAheadPoint(const TArray<FVector>& Points, const FVector& PathLocation,
		int32 NextPointIndex, double LookAheadDistance);
	float CalculateTurnFuel(double ErrorDegrees, float CurrentAngularVelocity,
		const FEngineSnapshot& Engine, float DeltaTime);
	FNavigationControl CalculateControl(const TArray<FVector>& Points, FNavigationState& State,
		const FEngineSnapshot& Engine, float DeltaTime);
	FNavigationControl CalculateSparseControl(const TArray<FVector>& Points, FNavigationState& State,
		const FEngineSnapshot& Engine, float DeltaTime, bool& bLaunchComplete,
		double& NextCorrectionSeconds);
	void LimitControlToFuel(FNavigationControl& Control, double AvailableFuelKg);
	double CalculatePredictionReserveUU(double RadiusUU, double RelativeSpeedUUPerSecond,
		double StepSeconds);
	double CalculateBodyPredictionReserveUU(const FPlannerBody& Body, double RelativeSpeedUUPerSecond,
		double StepSeconds);
	double CalculateShipToPlanetSpeedRisk(const FPlannerContext& Context, const FPlannerBody& Body,
		double ShipSpeedUUPerSecond);
	double CalculateTurnReserveUU(double RadiusUU, double RelativeSpeedUUPerSecond,
		const FEngineSnapshot& Engine);
	FPredictedStateSample InterpolatePredictedState(const TArray<FPredictedStateSample>& Samples,
		double TimeSeconds, int32& NextSampleIndex);
	FNavigationControl CalculateCorrectionControl(const FVector& DesiredVelocityChange,
		const FQuat& Orientation, const USpaceNavEngineComponent& Engine, float DeltaTime);
	void BroadcastControl(ASpaceNavPawn& Pawn, const FNavigationControl& Control);
	bool BuildPlannerContext(const ASpaceNavPawn& Pawn, const USpaceNavEngineComponent& Engine,
		const USpaceNavResourceStoreComponent& Resources, const FVector& Target,
		double PlanetSafetyBufferFraction, FPlannerContext& OutContext);
	int32 CountGuideTurns(const TArray<FVector>& Points);
	void BuildHierarchicalGuides(const FPlannerContext& Context, TArray<FHierarchicalGuide>& OutGuides, bool bLog);
	void PrependStartEscape(TArray<FVector>& Route, const TArray<FVector>& EscapePoints);
	bool IsRouteClear(const TArray<FVector>& Points, const TArray<FSphere>& Obstacles);
	bool IsSegmentClear(const FVector& Start, const FVector& End, const TArray<FSphere>& Obstacles);
	bool SimulateCandidate(const TArray<FVector>& Guidance, const FPlannerContext& Context,
		EPlanFamily Family, double CruiseSpeedFraction,
		FRouteCandidate& OutCandidate, FBlockedPath* OutBlocked = nullptr,
		FSimulationFailure* OutFailure = nullptr);
	void AddDistinctCandidate(TArray<FRouteCandidate>& Candidates, FRouteCandidate&& Candidate);
	bool AddDistinctPath(TArray<TArray<FVector>>& Paths, const TArray<FVector>& Path);
	double GetCandidateCost(const FRouteCandidate& Candidate, const FSpaceNavRouteWeightSet& Weights,
		double MinimumFuelKg, double MaximumFuelKg, double MinimumTimeSeconds, double MaximumTimeSeconds);
	void LogSelectedPlanDetails(const FRouteCandidate& Candidate, bool bLog);
	void SearchRoutes(const FPlannerContext& Context, FRouteSearchResults& OutResults, bool bLog);
	void BuildTimeCurveVariants(const FPlannerContext& Context, FRouteSearchResults& Results, bool bLog);
	const FRouteCandidate* FindRouteCandidate(const FRouteSearchResults& Results, int32 Index);
	void SelectTimeCurveCandidate(const FRouteSearchResults& Results,
		const FSpaceNavRouteWeightSet& ModeWeights, double SeparationUU,
		int32* OutIndices, double* OutCosts);

	void SnapshotPredictionSettings(const USpaceNavRouteOptimizationDataAsset& Settings,
		FPlannerContext& OutContext);
	bool ArePredictedPathsDistinct(const TArray<FVector>& First, const TArray<FVector>& Second,
		double SeparationUU);
	void SelectDistinctModeCandidates(const TArray<FRouteCandidate>& Candidates,
		const FSpaceNavRouteWeightSet* WeightSets, int32 FuelMaxTurns,
		double SeparationUU, int32* OutIndices, double* OutCosts);

}

struct FRouteSearchResults
{
	TArray<FRouteCandidate> Candidates;
	TArray<FRouteCandidate> TimeCandidates;
	double SnapshotGameSeconds = 0.0;
	double InitialFuelKg = 0.0;
	int32 AttemptedPaths = 0;
	int32 UnverifiedPaths = 0;
	int32 DetailedAttempts = 0;
	int32 DuplicatePlans = 0;
};

struct FRouteAutopilotState
{
	FPlannerContext SafetyContext;
	FScriptedGuardScratch SafetyScratch;
	FAutopilotTelemetry Telemetry;
	FAutopilotSnapshot Snapshot;
	TWeakObjectPtr<USpaceNavEngineComponent> Engine;
	FVector ExitVelocity = FVector::ZeroVector;
	TArray<FVector> FrameStartBodyPositions;
	TArray<FVector> FrameEndBodyPositions;
	int32 MotionSegmentIndex = 0;
};

USpaceNavRouteComponent::USpaceNavRouteComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = false;
}

void USpaceNavRouteComponent::BuildFuelEfficientRoute()
{
	BuildRouteForMode(ERouteMode::Fuel);
}

void USpaceNavRouteComponent::BuildTimeEfficientRoute()
{
	BuildRouteForMode(ERouteMode::Time);
}

void USpaceNavRouteComponent::BuildBalancedRoute()
{
	BuildRouteForMode(ERouteMode::Balanced);
}

void USpaceNavRouteComponent::BeginPlay()
{
	Super::BeginPlay();
}

void USpaceNavRouteComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	bSearchStopped = true;
	bHasPendingMode = false;
	FinishAutopilot(TEXT("end"));
	ResumeEngineAfterSearch();
	RoutePool.Reset();
	Super::EndPlay(EndPlayReason);
}

void USpaceNavRouteComponent::StartBackgroundSearch()
{
	if (bSearchStarted || bSearchStopped) return;
	bSearchStarted = true;
	const auto failSearch = [this]()
	{
		bSearchStarted = false;
		bSearchFinished = true;
		bHasPendingMode = false;
		ResumeEngineAfterSearch();
	};
	if (!CheckSettings()) { failSearch(); return; }
	LogDisplay(TEXT("Settings ready"));

	ASpaceNavPawn* pawn = Cast<ASpaceNavPawn>(GetOwner());
	if (pawn == nullptr)
	{
		UE_LOG(LogSpaceNavRoute, Error, TEXT("Failed: route owner is not a ship pawn"));
		failSearch();
		return;
	}
	USpaceNavEngineComponent* engine = pawn->FindComponentByClass<USpaceNavEngineComponent>();
	USpaceNavResourceStoreComponent* resources =
		pawn->FindComponentByClass<USpaceNavResourceStoreComponent>();
	if (engine == nullptr || resources == nullptr)
	{
		UE_LOG(LogSpaceNavRoute, Error, TEXT("Failed: ship parts missing"));
		failSearch();
		return;
	}
	LogDisplay(TEXT("Ship parts ready"));

	FVector targetLocation;
	if (!FindTargetLocation(targetLocation)) { failSearch(); return; }
	TArray<FSphere> obstacles;
	int32 startPlanetIndex = INDEX_NONE;
	double startPlanetRadius = 0.0;
	if (!CollectObstacles(obstacles, startPlanetIndex, startPlanetRadius))
	{
		failSearch();
		return;
	}
	TArray<FVector> escapePoints;
	if (!BuildStartEscape(targetLocation, obstacles, startPlanetIndex, startPlanetRadius, escapePoints))
	{
		failSearch();
		return;
	}
	FPlannerContext context;
	SnapshotPredictionSettings(*OptimizationSettings, context);
	if (!BuildPlannerContext(*pawn, *engine, *resources, targetLocation,
		PawnSettings->PlanetSafetyBufferFraction, context))
	{
		failSearch();
		return;
	}
	context.Obstacles = MoveTemp(obstacles);
	context.EscapePoints = MoveTemp(escapePoints);
	PausedEngine = engine;
	if (!bEnginePausedForSearch)
	{
		bEngineTickWasEnabled = engine->IsComponentTickEnabled();
		bEnginePausedForSearch = true;
		engine->SetComponentTickEnabled(false);
	}
	LogDisplay(TEXT("Ship held"));
	if (bLogRoute)
	{
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Physics ready: bodies=%d fuel=%.3f gravity=%.6g"),
			context.Bodies.Num(), context.InitialFuelKg, context.GravityCoefficient);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Safety gap R %.3g"), PawnSettings->PlanetSafetyBufferFraction);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Critical gap R %.3g"), context.CriticalBodyClearanceRadiusFraction);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Speed factor %.3g"), context.ShipToPlanetSpeedFactor);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Error %% %.3g"), context.ManeuverVelocityErrorFraction * 100.0);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Fuel reserve %% %.3g"), context.DesiredCorrectionFuelReserveFraction * 100.0);
	}
	LogDisplay(TEXT("Search started"));
	TWeakObjectPtr<USpaceNavRouteComponent> weakThis(this);
	const bool bLog = bLogRoute;
	const TSharedPtr<FRouteSearchResults, ESPMode::ThreadSafe> results = RoutePool;
	results->SnapshotGameSeconds = GetWorld()->GetTimeSeconds();
	Async(EAsyncExecution::ThreadPool, [weakThis, context = MoveTemp(context), bLog, results]() mutable
	{
		SearchRoutes(context, *results, bLog);
		BuildTimeCurveVariants(context, *results, bLog);
		AsyncTask(ENamedThreads::GameThread, [weakThis, results]() mutable
		{
			if (USpaceNavRouteComponent* route = weakThis.Get())
				route->OnSearchCompleted(results);
		});
	});
}

void USpaceNavRouteComponent::OnSearchCompleted(
	TSharedPtr<FRouteSearchResults, ESPMode::ThreadSafe> Results)
{
	if (bSearchStopped) return;
	bSearchStarted = false;
	if (Results != RoutePool)
	{
		StartBackgroundSearch();
		return;
	}
	bSearchFinished = true;
	if (bLogRoute)
	{
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Pool ready n%d"), RoutePool->Candidates.Num());
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Unverified %d"), RoutePool->UnverifiedPaths);
	}
	if (RoutePool->Candidates.IsEmpty())
	{
		bHasPendingMode = false;
		ResumeEngineAfterSearch();
		UE_LOG(LogSpaceNavRoute, Error, TEXT("No verified route"));
		return;
	}
	if (!CheckSettings())
	{
		bHasPendingMode = false;
		ResumeEngineAfterSearch();
		return;
	}
	const FSpaceNavRouteWeightSet weightSets[] = {OptimizationSettings->FuelEfficient,
		OptimizationSettings->TimeEfficient, OptimizationSettings->Balanced};
	double modeCosts[3] = {};
	const TArray<FVector>& referenceGuide = RoutePool->Candidates[0].Guidance;
	const double separationUU = OptimizationSettings->MinimumRouteSeparationFraction *
		FVector::Dist(referenceGuide[0], referenceGuide.Last());
	SelectDistinctModeCandidates(RoutePool->Candidates, weightSets,
		OptimizationSettings->FuelMaxTurns, separationUU, ModeCandidateIndices, modeCosts);
	SelectTimeCurveCandidate(*RoutePool, weightSets[1], separationUU, ModeCandidateIndices, modeCosts);
	int32 availableModes = 0;
	for (int32 modeIndex = 0; modeIndex < 3; ++modeIndex)
	{
		if (ModeCandidateIndices[modeIndex] != INDEX_NONE) ++availableModes;
		else UE_LOG(LogSpaceNavRoute, Warning, TEXT("Mode %d unavailable"), modeIndex);
	}
	if (bLogRoute) UE_LOG(LogSpaceNavRoute, Display, TEXT("Modes ready %d/3"), availableModes);
	if (!bHasPendingMode) { ResumeEngineAfterSearch(); return; }
	const ERouteMode mode = PendingMode;
	bHasPendingMode = false;
	const int32 selectedIndex = ModeCandidateIndices[static_cast<int32>(mode)];
	SelectedCandidateIndex = selectedIndex;
	if (selectedIndex == INDEX_NONE)
	{
		ResumeEngineAfterSearch();
		return;
	}
	const FRouteCandidate& selected = *FindRouteCandidate(*RoutePool, selectedIndex);
	GuidancePoints = selected.Guidance;
	RoutePoints = selected.PredictedPath;
	bRouteReady = true;
	if (bLogRoute)
	{
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Mode %d plan %d"), static_cast<int32>(mode), selectedIndex);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Family %s"),
			selected.Family == EPlanFamily::SparseBurns ? TEXT("Sparse") : TEXT("Corrected"));
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Selected cost %.4f"), modeCosts[static_cast<int32>(mode)]);
	}
	LogSelectedPlanDetails(selected, bLogRoute);
	DrawRoute();
}

void USpaceNavRouteComponent::BuildRouteForMode(ERouteMode Mode)
{
	if (bSearchStopped) return;
	FinishAutopilot(TEXT("reset"));
	if (RouteLineBatch != nullptr) RouteLineBatch->Flush();
	bRouteReady = false;
	NextRoutePointIndex = INDEX_NONE;
	SelectedCandidateIndex = INDEX_NONE;
	for (int32& index : ModeCandidateIndices) index = INDEX_NONE;
	RouteElapsedSeconds = 0.0;
	bHasPreviousPawnLocation = false;
	SetComponentTickEnabled(false);
	GuidancePoints.Reset();
	RoutePoints.Reset();
	RoutePool = MakeShared<FRouteSearchResults, ESPMode::ThreadSafe>();
	bSearchFinished = false;
	PendingMode = Mode;
	bHasPendingMode = true;
	if (Mode == ERouteMode::Fuel) LogDisplay(TEXT("Queued Fuel"));
	else if (Mode == ERouteMode::Time) LogDisplay(TEXT("Queued Time"));
	else LogDisplay(TEXT("Queued Balanced"));
	StartBackgroundSearch();
}

void USpaceNavRouteComponent::MoveRoad()
{
	if (AutopilotState.IsValid() || !bRouteReady || GuidancePoints.Num() < 2 || !RoutePool.IsValid()) return;
	const TSharedPtr<FRouteSearchResults, ESPMode::ThreadSafe> launchPool = RoutePool;
	const FRouteCandidate* selected = FindRouteCandidate(*launchPool, SelectedCandidateIndex);
	if (selected == nullptr) return;
	const FRouteCandidate& candidate = *selected;
	if (candidate.MotionSegments.IsEmpty()) return;
	ASpaceNavPawn* pawn = Cast<ASpaceNavPawn>(GetOwner());
	if (pawn == nullptr) return;
	USpaceNavEngineComponent* engine = pawn->FindComponentByClass<USpaceNavEngineComponent>();
	USpaceNavResourceStoreComponent* resources =
		pawn->FindComponentByClass<USpaceNavResourceStoreComponent>();
	if (engine == nullptr || resources == nullptr || engine->MaxSpeed <= 0.0f)
	{
		UE_LOG(LogSpaceNavRoute, Warning, TEXT("Auto invalid"));
		ResumeEngineAfterSearch();
		return;
	}
	if (!CheckSettings()) { ResumeEngineAfterSearch(); return; }
	const double validationStarted = FPlatformTime::Seconds();
	FPlannerContext launchContext;
	SnapshotPredictionSettings(*OptimizationSettings, launchContext);
	if (!BuildPlannerContext(*pawn, *engine, *resources, GuidancePoints.Last(),
		PawnSettings->PlanetSafetyBufferFraction, launchContext))
	{
		bRouteReady = false;
		ResumeEngineAfterSearch();
		return;
	}
	if (bLogRoute) UE_LOG(LogSpaceNavRoute, Display, TEXT("Plan age %.3gs"),
		FMath::Max(0.0, static_cast<double>(GetWorld()->GetTimeSeconds()) - launchPool->SnapshotGameSeconds));
	FScriptedGuardScratch launchScratch;
	FSimulationFailure launchFailure;
	const FScriptedPose startPose = EvaluateScriptedMotion(candidate.MotionSegments, 0.0);
	if (FVector::DistSquared(pawn->GetActorLocation(), startPose.Position) > FMath::Square(ArrivalDistance))
	{
		launchFailure.Reason = ESimulationFailureReason::InvalidInput;
		LogRouteSafetyResult(TEXT("Launch"), &launchFailure,
			(FPlatformTime::Seconds() - validationStarted) * 1000.0, bLogRoute);
		bRouteReady = false;
		ResumeEngineAfterSearch();
		return;
	}
	const bool bLaunchSafe = ValidateScriptedWindow(candidate, 0.0,
		candidate.FlightTimeSeconds, launchContext, 0.0, launchScratch, launchFailure);
	LogRouteSafetyResult(TEXT("Launch"), bLaunchSafe ? nullptr : &launchFailure,
		(FPlatformTime::Seconds() - validationStarted) * 1000.0, bLogRoute);
	if (bSearchStopped || RoutePool != launchPool) return;
	if (!bLaunchSafe)
	{
		bRouteReady = false;
		ResumeEngineAfterSearch();
		return;
	}
	FinishAutopilot(TEXT("reset"));
	AutopilotState = MakeShared<FRouteAutopilotState>();
	AutopilotState->SafetyContext = MoveTemp(launchContext);
	AutopilotState->SafetyScratch = MoveTemp(launchScratch);
	AutopilotState->Engine = engine;
	AutopilotState->ExitVelocity = engine->GetTotalLinearVelocity();
	ResetAutopilotTelemetry(AutopilotState->Telemetry, resources->Resources.Fuel);
	RouteElapsedSeconds = 0.0;
	const FScriptedMotionSegment& firstSegment = candidate.MotionSegments[0];
	const double firstFuelKg = firstSegment.bCurved
		? firstSegment.CurveRemainingFuelKg : GetScriptedFuelKg(firstSegment.Fuel);
	if (firstFuelKg > static_cast<double>(resources->Resources.Fuel) +
		GetScriptedFuelToleranceKg(AutopilotState->Telemetry.InitialFuelKg))
	{
		FinishAutopilot(TEXT("fuel"));
		ResumeEngineAfterSearch();
		return;
	}
	engine->AddTickPrerequisiteComponent(this);
	engine->BeginScriptedFlight();
	const TSharedPtr<FRouteAutopilotState> startedFlight = AutopilotState;
	UpdateScriptedPose(*pawn, *engine, *resources);
	if (AutopilotState != startedFlight) return;
	CaptureCollisionBodies();
	bImpactLogged = false;
	PreviousPawnLocation = pawn->GetActorLocation();
	bHasPreviousPawnLocation = true;
	SetComponentTickEnabled(true);
	ResumeEngineAfterSearch();
	LogDisplay(TEXT("Auto scripted"));
}

void USpaceNavRouteComponent::TickComponent(float DeltaTime, ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	AdvanceMoveRoad(DeltaTime);
}

bool USpaceNavRouteComponent::CheckSettings() const
{
	if (PawnSettings == nullptr)
	{
		UE_LOG(LogSpaceNavRoute, Error, TEXT("Failed: pawn settings missing"));
		return false;
	}
	if (OptimizationSettings == nullptr)
	{
		UE_LOG(LogSpaceNavRoute, Error, TEXT("Failed: optimization settings missing"));
		return false;
	}
	const FSpaceNavRouteWeightSet* weightSets[] = {
		&OptimizationSettings->FuelEfficient, &OptimizationSettings->TimeEfficient,
		&OptimizationSettings->Balanced};
	const TCHAR* weightNames[] = {TEXT("Fuel"), TEXT("Time"), TEXT("Balanced")};
	for (int32 weightIndex = 0; weightIndex < static_cast<int32>(UE_ARRAY_COUNT(weightSets)); ++weightIndex)
	{
		const FSpaceNavRouteWeightSet* weights = weightSets[weightIndex];
		const double sum = weights->FuelWeight + weights->TimeWeight + weights->RiskWeight;
		if (!FMath::IsFinite(sum) || sum <= 0.0 || weights->FuelWeight < 0.0 ||
			weights->TimeWeight < 0.0 || weights->RiskWeight < 0.0)
		{
			UE_LOG(LogSpaceNavRoute, Error,
				TEXT("Failed: %s weights invalid fuel=%.6g time=%.6g risk=%.6g"),
				weightNames[weightIndex], weights->FuelWeight, weights->TimeWeight,
				weights->RiskWeight);
			return false;
		}
	}

	if (OptimizationSettings->FuelMaxTurns < 0 || OptimizationSettings->FuelMaxTurns > 2 ||
		OptimizationSettings->MaxGuideCandidates <= 0 || OptimizationSettings->MaxDetailedCandidates <= 0 ||
		!FMath::IsFinite(OptimizationSettings->MinimumRouteSeparationFraction) ||
		OptimizationSettings->MinimumRouteSeparationFraction < 0.0 ||
		OptimizationSettings->MinimumRouteSeparationFraction > 1.0 ||
		OptimizationSettings->CruiseSpeedFractions.IsEmpty())
	{
		UE_LOG(LogSpaceNavRoute, Error, TEXT("Invalid prediction setup"));
		return false;
	}
	for (const double fraction : OptimizationSettings->CruiseSpeedFractions)
	{
		if (FMath::IsFinite(fraction) && fraction > 0.0 && fraction <= 1.0) continue;
		UE_LOG(LogSpaceNavRoute, Error, TEXT("Invalid cruise fraction"));
		return false;
	}

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

	UE_LOG(LogSpaceNavRoute, Error, TEXT("Failed: target marker missing"));
	return false;
}

bool USpaceNavRouteComponent::CollectObstacles(TArray<FSphere>& OutObstacles,
	int32& OutStartPlanetIndex, double& OutStartPlanetRadius) const
{
	const FVector startLocation = GetOwner()->GetActorLocation();
	int32 skippedStartBodies = 0;
	OutStartPlanetIndex = INDEX_NONE;
	OutStartPlanetRadius = 0.0;

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
		const double planetRadius = planetIterator->RadiusMeters * UnrealUnitsPerMeter;
		if (!FMath::IsFinite(planetRadius) || planetRadius <= 0.0) continue;
		const FVector center = planetIterator->GetActorLocation();
		const double startDistanceSquared = FVector::DistSquared(startLocation, center);
		if (startDistanceSquared <= planetRadius * planetRadius)
		{
			UE_LOG(LogSpaceNavRoute, Error, TEXT("Failed: start inside planet %s"),
				*planetIterator->GetName());
			return false;
		}
		OutObstacles.Emplace(center, planetRadius);
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

bool USpaceNavRouteComponent::BuildStartEscape(const FVector& TargetLocation,
	const TArray<FSphere>& Obstacles, int32 StartPlanetIndex, double StartPlanetRadius,
	TArray<FVector>& OutEscapePoints) const
{
	OutEscapePoints.Reset();
	if (StartPlanetIndex == INDEX_NONE)
	{
		LogDisplay(TEXT("Start escape not needed"));
		return true;
	}

	const FVector startLocation = GetOwner()->GetActorLocation();
	const FSphere& startZone = Obstacles[StartPlanetIndex];
	const FVector fromCenter = startLocation - startZone.Center;
	const double startDistance = fromCenter.Size();
	const FVector outwardDirection = fromCenter / startDistance;
	const FVector towardTarget = TargetLocation - startLocation;
	const FVector lateralDirection = (towardTarget - outwardDirection *
		FVector::DotProduct(towardTarget, outwardDirection)).GetSafeNormal();
	const double outwardDistance = startZone.W - startDistance + UnrealUnitsPerMeter;
	const double lateralDistance = outwardDistance * 0.5;

	OutEscapePoints.Reserve(CornerSampleCount + 1);
	OutEscapePoints.Add(startLocation);
	for (int32 sampleIndex = 1; sampleIndex <= CornerSampleCount; ++sampleIndex)
	{
		const double fraction = static_cast<double>(sampleIndex) / CornerSampleCount;
		OutEscapePoints.Add(startLocation + outwardDirection * (outwardDistance * fraction) +
			lateralDirection * (lateralDistance * fraction * fraction));
	}

	TArray<FSphere> escapeObstacles = Obstacles;
	escapeObstacles[StartPlanetIndex].W = StartPlanetRadius;
	if (!IsRouteClear(OutEscapePoints, escapeObstacles))
	{
		UE_LOG(LogSpaceNavRoute, Error, TEXT("Failed: start escape intersects an obstacle"));
		return false;
	}
	if (bLogRoute)
	{
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Escape %d pts"), OutEscapePoints.Num());
	}
	return true;
}

namespace
{
void PrependStartEscape(TArray<FVector>& Route, const TArray<FVector>& EscapePoints)
{
	if (EscapePoints.IsEmpty()) return;
	TArray<FVector> fullRoute;
	fullRoute.Reserve(EscapePoints.Num() + Route.Num() - 1);
	fullRoute.Append(EscapePoints);
	for (int32 pointIndex = 1; pointIndex < Route.Num(); ++pointIndex)
	{
		fullRoute.Add(Route[pointIndex]);
	}
	Route = MoveTemp(fullRoute);
}
}

void USpaceNavRouteComponent::DrawRoute()
{
	if (RouteLineBatch == nullptr)
	{
		RouteLineBatch = NewObject<ULineBatchComponent>(GetOwner(), NAME_None, RF_Transient);
		RouteLineBatch->RegisterComponent();
		RouteLineBatch->SetComponentTickEnabled(false);
	}
	for (int32 pointIndex = 1; pointIndex < RoutePoints.Num(); ++pointIndex)
	{
		RouteLineBatch->DrawLine(RoutePoints[pointIndex - 1], RoutePoints[pointIndex],
			FLinearColor(FColor::Cyan), 0, 30.0f, -1.0f);
	}
	LogDisplay(TEXT("Done"));
}

namespace
{

int32 CountGuideTurns(const TArray<FVector>& Points)
{
	int32 bendCount = 0;
	for (int32 pointIndex = 1; pointIndex + 1 < Points.Num(); ++pointIndex)
	{
		const FVector incoming = (Points[pointIndex] - Points[pointIndex - 1]).GetSafeNormal();
		const FVector outgoing = (Points[pointIndex + 1] - Points[pointIndex]).GetSafeNormal();
		if (incoming.IsNearlyZero() || outgoing.IsNearlyZero()) continue;
		if (FVector::DotProduct(incoming, outgoing) < 1.0 - 1.e-6) ++bendCount;
	}
	return bendCount;
}

struct FCoarseEncounter
{
	int32 BodyIndex = INDEX_NONE;
	int32 SegmentIndex = INDEX_NONE;
	double ArrivalSeconds = 0.0;
	FVector Center = FVector::ZeroVector;
	double RadiusUU = 0.0;
};

struct FCoarseCorridor
{
	EGuideStyle Style = EGuideStyle::Fuel;
	TArray<FVector> Points;
	TArray<int32> EncounterBodies;
	int32 RefinementDepth = 0;
	double EstimatedFuelKg = 0.0;
	double EstimatedTimeSeconds = 0.0;
	double EstimatedRiskCost = 0.0;
};

double GetCoarseCruiseSpeed(const FPlannerContext& Context, EGuideStyle Style)
{
	double fraction = 1.0;
	if (Style == EGuideStyle::Fuel)
	{
		for (const double candidateFraction : Context.CruiseSpeedFractions)
		{
			if (candidateFraction <= 0.0) continue;
			fraction = candidateFraction;
			break;
		}
	}
	else if (Style == EGuideStyle::Balanced && !Context.CruiseSpeedFractions.IsEmpty())
	{
		fraction = Context.CruiseSpeedFractions[Context.CruiseSpeedFractions.Num() / 2];
	}
	return FMath::Max(1.0, static_cast<double>(Context.Engine.MaxSpeed) * fraction);
}

double GetEscapeLength(const FPlannerContext& Context)
{
	double length = 0.0;
	for (int32 pointIndex = 1; pointIndex < Context.EscapePoints.Num(); ++pointIndex)
	{
		length += FVector::Dist(Context.EscapePoints[pointIndex - 1], Context.EscapePoints[pointIndex]);
	}
	return length;
}

void EstimateCoarseCorridor(const FPlannerContext& Context, FCoarseCorridor& Corridor)
{
	const double cruiseSpeed = GetCoarseCruiseSpeed(Context, Corridor.Style);
	const FVector initialVelocity = Context.InitialState.EngineVelocity + Context.InitialState.GravityVelocity;
	const double thrustImpulse = FMath::Max(1.0, static_cast<double>(Context.Engine.ThrustAcceleration));
	const double launchSeconds = FMath::Max(0.0, cruiseSpeed - initialVelocity.Size()) / thrustImpulse;
	double distance = GetEscapeLength(Context);
	double elapsedSeconds = launchSeconds + distance / cruiseSpeed;
	double estimatedFuelKg = 0.0;
	double accumulatedRisk = 0.0;
	double riskDistance = 0.0;
	if (Corridor.Points.Num() > 1)
	{
		const FVector firstDirection = (Corridor.Points[1] - Corridor.Points[0]).GetSafeNormal();
		estimatedFuelKg = (firstDirection * cruiseSpeed - initialVelocity).Size() / thrustImpulse;
	}
	for (int32 pointIndex = 1; pointIndex < Corridor.Points.Num(); ++pointIndex)
	{
		const FVector segment = Corridor.Points[pointIndex] - Corridor.Points[pointIndex - 1];
		const double segmentLength = segment.Size();
		const FVector segmentMiddle = (Corridor.Points[pointIndex] + Corridor.Points[pointIndex - 1]) * 0.5;
		const double middleSeconds = elapsedSeconds + 0.5 * segmentLength / cruiseSpeed;
		double segmentRisk = 0.0;
		for (const FPlannerBody& body : Context.Bodies)
		{
			const double radius = FMath::Max(body.RadiusUU, body.AvoidRadiusUU);
			const double centerDistance = FVector::Dist(segmentMiddle, body.LocationAfter(middleSeconds));
			const double distanceRisk = radius / FMath::Max(radius, centerDistance);
			const double speedRisk = CalculateShipToPlanetSpeedRisk(Context, body, cruiseSpeed);
			segmentRisk = FMath::Max(segmentRisk, distanceRisk +
				(1.0 - distanceRisk) * distanceRisk * speedRisk * 0.25);
		}
		accumulatedRisk += segmentRisk * segmentLength;
		riskDistance += segmentLength;
		distance += segmentLength;
		elapsedSeconds += segmentLength / cruiseSpeed;
		if (pointIndex + 1 >= Corridor.Points.Num() || segment.IsNearlyZero()) continue;
		const FVector outgoing = (Corridor.Points[pointIndex + 1] - Corridor.Points[pointIndex]).GetSafeNormal();
		if (outgoing.IsNearlyZero()) continue;
		const double cosine = FMath::Clamp(FVector::DotProduct(segment.GetSafeNormal(), outgoing), -1.0, 1.0);
		const double turnAngle = FMath::Acos(cosine);
		estimatedFuelKg += 2.0 * cruiseSpeed * FMath::Sin(turnAngle * 0.5) / thrustImpulse;
		const double turnRate = FMath::Max(1.0, static_cast<double>(Context.Engine.MaxTurnSpeed));
		const double turnSeconds = FMath::RadiansToDegrees(turnAngle) / turnRate;
		elapsedSeconds += turnSeconds;
		if (Context.Engine.TurnImpulsePerFuelUnit > 0.0f)
		{
			estimatedFuelKg += turnRate / Context.Engine.TurnImpulsePerFuelUnit * FMath::Min(1.0, turnSeconds);
		}
	}
	Corridor.EstimatedFuelKg = estimatedFuelKg;
	Corridor.EstimatedTimeSeconds = elapsedSeconds;
	const double distanceRisk = riskDistance > 0.0 ? accumulatedRisk / riskDistance : 0.0;
	const double desiredReserve = estimatedFuelKg * Context.DesiredCorrectionFuelReserveFraction;
	const double availableReserve = FMath::Max(0.0, Context.InitialFuelKg - estimatedFuelKg);
	const double reserveRisk = desiredReserve > 0.0
		? (availableReserve > 0.0 ? FMath::Clamp(desiredReserve / availableReserve, 0.0, 1.0) : 1.0) : 0.0;
	Corridor.EstimatedRiskCost = distanceRisk + (1.0 - distanceRisk) * reserveRisk * 0.25;
}

double GetCoarseCorridorCost(const FPlannerContext& Context, const FCoarseCorridor& Corridor)
{
	const FSpaceNavRouteWeightSet& weights = Context.ModeWeights[static_cast<int32>(Corridor.Style)];
	const double fuelScale = FMath::Max(1.0, Context.InitialFuelKg);
	const double directTime = FMath::Max(1.0, FVector::Dist(Context.InitialState.Position,
		Context.Target) / FMath::Max(1.0, static_cast<double>(Context.Engine.MaxSpeed)));
	return weights.FuelWeight * Corridor.EstimatedFuelKg / fuelScale +
		weights.TimeWeight * Corridor.EstimatedTimeSeconds / directTime +
		weights.RiskWeight * Corridor.EstimatedRiskCost;
}

bool FindCoarseEncounter(const FPlannerContext& Context, const FCoarseCorridor& Corridor,
	FCoarseEncounter& OutEncounter)
{
	const double cruiseSpeed = GetCoarseCruiseSpeed(Context, Corridor.Style);
	const FVector initialVelocity = Context.InitialState.EngineVelocity + Context.InitialState.GravityVelocity;
	const double initialSeconds = GetEscapeLength(Context) / cruiseSpeed +
		FMath::Max(0.0, cruiseSpeed - initialVelocity.Size()) /
		FMath::Max(1.0, static_cast<double>(Context.Engine.ThrustAcceleration));
	for (int32 bodyKind = 0; bodyKind < 2; ++bodyKind)
	{
		double elapsedSeconds = initialSeconds;
		for (int32 segmentIndex = 1; segmentIndex < Corridor.Points.Num(); ++segmentIndex)
		{
			const FVector start = Corridor.Points[segmentIndex - 1];
			const FVector segment = Corridor.Points[segmentIndex] - start;
			const double lengthSquared = segment.SizeSquared();
			const double segmentSeconds = FMath::Sqrt(lengthSquared) / cruiseSpeed;
			double earliestFraction = TNumericLimits<double>::Max();
			for (int32 bodyIndex = 0; bodyIndex < Context.Bodies.Num(); ++bodyIndex)
			{
				const FPlannerBody& body = Context.Bodies[bodyIndex];
				if (body.bPlanet != (bodyKind == 1)) continue;
				FVector center = body.LocationAfter(elapsedSeconds + segmentSeconds * 0.5);
				double fraction = lengthSquared > 0.0 ? FMath::Clamp(
					FVector::DotProduct(center - start, segment) / lengthSquared, 0.0, 1.0) : 0.0;
				center = body.LocationAfter(elapsedSeconds + segmentSeconds * fraction);
				fraction = lengthSquared > 0.0 ? FMath::Clamp(
					FVector::DotProduct(center - start, segment) / lengthSquared, 0.0, 1.0) : 0.0;
				const double arrivalSeconds = elapsedSeconds + segmentSeconds * fraction;
				center = body.LocationAfter(arrivalSeconds);
				const double safetyRadius = body.bPlanet
					? FMath::Max(body.RadiusUU, body.AvoidRadiusUU) + body.RadiusUU * MinimumBodyReserveFraction
					: body.RadiusUU;
				const double radius = FMath::Max(safetyRadius, body.CriticalRadiusUU);
				if (FVector::DistSquared(start + segment * fraction, center) >= FMath::Square(radius)) continue;
				if (fraction >= earliestFraction) continue;
				earliestFraction = fraction;
				OutEncounter.BodyIndex = bodyIndex;
				OutEncounter.SegmentIndex = segmentIndex;
				OutEncounter.ArrivalSeconds = arrivalSeconds;
				OutEncounter.Center = center;
				OutEncounter.RadiusUU = radius;
			}
			if (OutEncounter.BodyIndex != INDEX_NONE) return true;
			elapsedSeconds += segmentSeconds;
		}
	}
	return false;
}


bool AddCoarseCorridor(const FPlannerContext& Context, FCoarseCorridor&& Corridor,
	TArray<TArray<FVector>>& SeenPaths, TArray<FCoarseCorridor>& Frontier,
	int32 StyleBudget, int32& GeneratedCount)
{
	if (GeneratedCount >= StyleBudget || Corridor.Points.Num() < 2) return false;
	if (Corridor.Style == EGuideStyle::Fuel)
	{
		TArray<FVector> fullPoints = Corridor.Points;
		PrependStartEscape(fullPoints, Context.EscapePoints);
		if (CountGuideTurns(fullPoints) > Context.FuelMaxTurns) return false;
	}
	if (!AddDistinctPath(SeenPaths, Corridor.Points)) return false;
	EstimateCoarseCorridor(Context, Corridor);
	Frontier.Add(MoveTemp(Corridor));
	++GeneratedCount;
	return true;
}

bool HasFixedStarIntersection(const FPlannerContext& Context, const TArray<FVector>& Points)
{
	for (int32 pointIndex = 1; pointIndex < Points.Num(); ++pointIndex)
	{
		const FVector start = Points[pointIndex - 1];
		const FVector segment = Points[pointIndex] - start;
		const double lengthSquared = segment.SizeSquared();
		for (const FPlannerBody& body : Context.Bodies)
		{
			if (body.bPlanet) continue;
			const double fraction = lengthSquared > 0.0 ? FMath::Clamp(
				FVector::DotProduct(body.FixedCenter - start, segment) / lengthSquared, 0.0, 1.0) : 0.0;
			const double radius = body.RadiusUU + CalculateBodyPredictionReserveUU(body, 0.0, 0.0);
			if (FVector::DistSquared(start + segment * fraction, body.FixedCenter) < FMath::Square(radius)) return true;
		}
	}
	return false;
}

void SeedCoarseCorridors(const FPlannerContext& Context, EGuideStyle Style,
	TArray<TArray<FVector>>& SeenPaths, TArray<FCoarseCorridor>& Frontier,
	int32 StyleBudget, int32& GeneratedCount)
{
	if (StyleBudget <= 0) return;
	const FVector start = Context.EscapePoints.IsEmpty()
		? Context.InitialState.Position : Context.EscapePoints.Last();
	const FVector direct = Context.Target - start;
	const FVector forward = direct.GetSafeNormal();
	FVector lateral = FVector::CrossProduct(forward, FVector::UpVector).GetSafeNormal();
	if (lateral.IsNearlyZero()) lateral = FVector::CrossProduct(forward, FVector::RightVector).GetSafeNormal();
	const FVector secondLateral = FVector::CrossProduct(forward, lateral).GetSafeNormal();
	TArray<FCoarseCorridor> seeds;
	FCoarseCorridor& directCorridor = seeds.AddDefaulted_GetRef();
	directCorridor.Style = Style;
	directCorridor.Points.Add(start);
	directCorridor.Points.Add(Context.Target);
	double corridorWidth = FMath::Max(ArrivalDistance, direct.Size() * 0.1);
	const FPlannerBody* blockingStar = nullptr;
	for (const FPlannerBody& body : Context.Bodies)
	{
		if (body.bPlanet || forward.IsNearlyZero()) continue;
		const double fraction = FMath::Clamp(FVector::DotProduct(body.FixedCenter - start, direct) /
			direct.SizeSquared(), 0.0, 1.0);
		const double gap = FVector::Dist(start + direct * fraction, body.FixedCenter);
		if (gap < body.RadiusUU + CalculateBodyPredictionReserveUU(body, 0.0, 0.0) &&
			blockingStar == nullptr) blockingStar = &body;
		if (gap > body.AvoidRadiusUU * 3.0) continue;
		corridorWidth = FMath::Max(corridorWidth, body.AvoidRadiusUU * 2.0);
	}
	const FVector sides[] = {lateral, -lateral, secondLateral, -secondLateral};
	const double widthScales[] = {1.0, 2.0};
	if (blockingStar != nullptr)
	{
		const FVector startOutward = (start - blockingStar->FixedCenter).GetSafeNormal();
		const FVector targetOutward = (Context.Target - blockingStar->FixedCenter).GetSafeNormal();
		const double radius = FMath::Max(corridorWidth,
			(blockingStar->RadiusUU + CalculateBodyPredictionReserveUU(*blockingStar, 0.0, 0.0)) * 2.0);
		for (const FVector& side : sides)
		{
			FCoarseCorridor& corridor = seeds.AddDefaulted_GetRef();
			corridor.Style = Style;
			corridor.Points.Add(start);
			corridor.Points.Add(start + startOutward * radius + side * radius);
			corridor.Points.Add(Context.Target + targetOutward * radius + side * radius);
			corridor.Points.Add(Context.Target);
		}
	}
	if (!forward.IsNearlyZero())
	{
		for (const double widthScale : widthScales)
		{
			for (const FVector& side : sides)
			{
				const FVector offset = side * corridorWidth * widthScale;
				for (int32 cornerCount = 1; cornerCount <= 2; ++cornerCount)
				{
					FCoarseCorridor& corridor = seeds.AddDefaulted_GetRef();
					corridor.Style = Style;
					corridor.Points.Add(start);
					if (cornerCount == 1) corridor.Points.Add(start + direct * 0.5 + offset);
					else
					{
						corridor.Points.Add(start + direct * 0.25 + offset);
						corridor.Points.Add(start + direct * 0.75 + offset);
					}
					corridor.Points.Add(Context.Target);
				}
			}
		}
	}
	int32 reservedIndex = INDEX_NONE;
	double reservedCost = TNumericLimits<double>::Max();
	for (int32 seedIndex = 0; seedIndex < seeds.Num(); ++seedIndex)
	{
		FCoarseCorridor& seed = seeds[seedIndex];
		if (HasFixedStarIntersection(Context, seed.Points)) continue;
		if (Style == EGuideStyle::Fuel)
		{
			TArray<FVector> fullPoints = seed.Points;
			PrependStartEscape(fullPoints, Context.EscapePoints);
			if (CountGuideTurns(fullPoints) > Context.FuelMaxTurns) continue;
		}
		EstimateCoarseCorridor(Context, seed);
		const double cost = GetCoarseCorridorCost(Context, seed);
		if (cost >= reservedCost) continue;
		reservedIndex = seedIndex;
		reservedCost = cost;
	}
	if (reservedIndex != INDEX_NONE)
	{
		AddCoarseCorridor(Context, MoveTemp(seeds[reservedIndex]), SeenPaths, Frontier,
			StyleBudget, GeneratedCount);
	}
	for (int32 seedIndex = 0; seedIndex < seeds.Num() && GeneratedCount < StyleBudget; ++seedIndex)
	{
		if (seedIndex == reservedIndex) continue;
		AddCoarseCorridor(Context, MoveTemp(seeds[seedIndex]), SeenPaths, Frontier,
			StyleBudget, GeneratedCount);
	}
}


void RefineCoarseEncounter(const FPlannerContext& Context, const FCoarseCorridor& Corridor,
	const FCoarseEncounter& Encounter, TArray<TArray<FVector>>& SeenPaths,
	TArray<FCoarseCorridor>& Frontier, int32 StyleBudget, int32& GeneratedCount)
{
	if (Corridor.RefinementDepth >= 4 || GeneratedCount >= StyleBudget) return;
	const FVector incoming = (Corridor.Points[Encounter.SegmentIndex] -
		Corridor.Points[Encounter.SegmentIndex - 1]).GetSafeNormal();
	if (incoming.IsNearlyZero()) return;
	FVector lateral = FVector::CrossProduct(incoming, FVector::UpVector).GetSafeNormal();
	if (lateral.IsNearlyZero()) lateral = FVector::CrossProduct(incoming, FVector::RightVector).GetSafeNormal();
	const FVector secondLateral = FVector::CrossProduct(incoming, lateral).GetSafeNormal();
	const FVector sides[] = {lateral, -lateral, secondLateral, -secondLateral};
	const double baseScale = Corridor.Style == EGuideStyle::Fuel ? 2.5
		: Corridor.Style == EGuideStyle::Time ? 1.35 : 1.8;
	const double radiusScales[] = {baseScale, baseScale * 1.75};
	TArray<FVector> fullPoints = Corridor.Points;
	PrependStartEscape(fullPoints, Context.EscapePoints);
	const bool bShiftFuelCorners = Corridor.Style == EGuideStyle::Fuel &&
		CountGuideTurns(fullPoints) >= Context.FuelMaxTurns && Corridor.Points.Num() > 2;
	for (const double radiusScale : radiusScales)
	{
		for (const FVector& side : sides)
		{
			for (int32 passPointCount = 1; passPointCount <= 2; ++passPointCount)
			{
				if (GeneratedCount >= StyleBudget) return;
				FCoarseCorridor refined = Corridor;
				++refined.RefinementDepth;
				refined.EncounterBodies.Add(Encounter.BodyIndex);
				const double radius = Encounter.RadiusUU * radiusScale;
				const FVector passCenter = Context.Bodies[Encounter.BodyIndex].LocationAfter(Encounter.ArrivalSeconds);
				if (bShiftFuelCorners)
				{
					if (passPointCount == 1)
					{
						const int32 cornerIndex = FMath::Clamp(Encounter.SegmentIndex, 1, refined.Points.Num() - 2);
						refined.Points[cornerIndex] = passCenter + side * radius;
					}
					else
					{
						if (refined.Points.Num() < 4) continue;
						refined.Points[1] = passCenter + side * radius - incoming * radius * 0.5;
						refined.Points[refined.Points.Num() - 2] = passCenter + side * radius + incoming * radius * 0.5;
					}
				}
				else if (passPointCount == 1)
				{
					refined.Points.Insert(passCenter + side * radius, Encounter.SegmentIndex);
				}
				else
				{
					refined.Points.Insert(passCenter + side * radius - incoming * radius * 0.5,
						Encounter.SegmentIndex);
					refined.Points.Insert(passCenter + side * radius + incoming * radius * 0.5,
						Encounter.SegmentIndex + 1);
				}
				AddCoarseCorridor(Context, MoveTemp(refined), SeenPaths, Frontier, StyleBudget, GeneratedCount);
			}
		}
	}
}


void BuildHierarchicalGuides(const FPlannerContext& Context, TArray<FHierarchicalGuide>& OutGuides, bool bLog)
{
	OutGuides.Reset();
	double coarseSeconds = 0.0;
	double refineSeconds = 0.0;
	int32 coarseCount = 0;
	int32 encounterCount = 0;
	const EGuideStyle styles[] = {EGuideStyle::Fuel, EGuideStyle::Time, EGuideStyle::Balanced};
	const int32 guideBudget = FMath::Max(0, Context.MaxGuideCandidates);
	for (int32 styleIndex = 0; styleIndex < 3; ++styleIndex)
	{
		const int32 styleBudget = guideBudget / 3 + (styleIndex < guideBudget % 3 ? 1 : 0);
		TArray<FCoarseCorridor> frontier;
		TArray<TArray<FVector>> seenPaths;
		int32 generatedCount = 0;
		const double coarseStarted = FPlatformTime::Seconds();
		SeedCoarseCorridors(Context, styles[styleIndex], seenPaths, frontier, styleBudget, generatedCount);
		coarseSeconds += FPlatformTime::Seconds() - coarseStarted;
		coarseCount += generatedCount;
		const double refineStarted = FPlatformTime::Seconds();
		while (!frontier.IsEmpty())
		{
			frontier.Sort([&Context](const FCoarseCorridor& first, const FCoarseCorridor& second)
			{
				return GetCoarseCorridorCost(Context, first) > GetCoarseCorridorCost(Context, second);
			});
			FCoarseCorridor corridor = MoveTemp(frontier.Last());
			frontier.Pop();
			FCoarseEncounter encounter;
			if (FindCoarseEncounter(Context, corridor, encounter))
			{
				++encounterCount;
				RefineCoarseEncounter(Context, corridor, encounter, seenPaths,
					frontier, styleBudget, generatedCount);
				if (!Context.Bodies[encounter.BodyIndex].bPlanet) continue;
			}
			FHierarchicalGuide& guide = OutGuides.AddDefaulted_GetRef();
			guide.Style = corridor.Style;
			guide.Points = MoveTemp(corridor.Points);
			PrependStartEscape(guide.Points, Context.EscapePoints);
			guide.BendCount = CountGuideTurns(guide.Points);
			guide.EstimatedFuelKg = corridor.EstimatedFuelKg;
			guide.EstimatedTimeSeconds = corridor.EstimatedTimeSeconds;
			guide.EstimatedRiskCost = corridor.EstimatedRiskCost;
		}
		refineSeconds += FPlatformTime::Seconds() - refineStarted;
	}
	if (bLog)
	{
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Coarse %d %.1fms"), coarseCount, coarseSeconds * 1000.0);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Refine %d %.1fms"), encounterCount, refineSeconds * 1000.0);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Guides %d"), OutGuides.Num());
	}
}
bool IsRouteClear(const TArray<FVector>& Points,
	const TArray<FSphere>& Obstacles)
{
	if (Points.Num() < 2) return false;
	for (int32 pointIndex = 1; pointIndex < Points.Num(); ++pointIndex)
	{
		if (!IsSegmentClear(Points[pointIndex - 1], Points[pointIndex], Obstacles)) return false;
	}
	return true;
}

bool IsSegmentClear(const FVector& Start, const FVector& End,
	const TArray<FSphere>& Obstacles)
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
}

void USpaceNavRouteComponent::AdvanceMoveRoad(float DeltaTime)
{
	if (!AutopilotState.IsValid() || DeltaTime <= 0.0f) return;
	ASpaceNavPawn* pawn = Cast<ASpaceNavPawn>(GetOwner());
	if (pawn == nullptr) { FinishAutopilot(TEXT("invalid")); return; }
	USpaceNavEngineComponent* engine = pawn->FindComponentByClass<USpaceNavEngineComponent>();
	USpaceNavResourceStoreComponent* resources =
		pawn->FindComponentByClass<USpaceNavResourceStoreComponent>();
	if (engine == nullptr || resources == nullptr)
	{
		FinishAutopilot(TEXT("invalid"));
		return;
	}
	ExecuteScriptedFlight(DeltaTime, *pawn, *engine, *resources);
}

void USpaceNavRouteComponent::ExecuteScriptedFlight(float DeltaTime, ASpaceNavPawn& Pawn,
	USpaceNavEngineComponent& Engine, USpaceNavResourceStoreComponent& Resources)
{
	const FRouteCandidate* candidate = RoutePool.IsValid()
		? FindRouteCandidate(*RoutePool, SelectedCandidateIndex) : nullptr;
	if (candidate == nullptr)
	{
		FinishAutopilot(TEXT("invalid"));
		return;
	}
	const double finishSeconds = candidate->FlightTimeSeconds;
	const double endSeconds = FMath::Min(RouteElapsedSeconds + DeltaTime, finishSeconds);
	if (!AdvanceScriptedSegments(endSeconds, DeltaTime, Pawn, Engine, Resources)) return;
	FlushAutopilotTelemetry(AutopilotState->Telemetry, AutopilotState->Snapshot, bLogRoute);
	if (RouteElapsedSeconds >= finishSeconds) FinishAutopilot(TEXT("arrived"));
}

bool USpaceNavRouteComponent::AdvanceScriptedSegments(double EndSeconds, float FrameDeltaTime,
	ASpaceNavPawn& Pawn, USpaceNavEngineComponent& Engine, USpaceNavResourceStoreComponent& Resources)
{
	const TSharedPtr<FRouteAutopilotState> flightState = AutopilotState;
	const TSharedPtr<FRouteSearchResults, ESPMode::ThreadSafe> pool = RoutePool;
	const FRouteCandidate& flightCandidate = *FindRouteCandidate(*pool, SelectedCandidateIndex);
	const TArray<FScriptedMotionSegment>& segments = flightCandidate.MotionSegments;
	RefreshRouteSafetyContext(flightState->SafetyContext);
	FSimulationFailure guardFailure;
	const double guardEndSeconds = FMath::Min(flightCandidate.FlightTimeSeconds,
		FMath::Max(EndSeconds, RouteElapsedSeconds + LiveThreatLookAheadSeconds));
	if (!ValidateScriptedWindow(flightCandidate, RouteElapsedSeconds, guardEndSeconds,
		flightState->SafetyContext, FrameDeltaTime, flightState->SafetyScratch,
		guardFailure, flightState->MotionSegmentIndex))
	{
		LogRouteSafetyResult(TEXT("Guard"), &guardFailure, 0.0, bLogRoute);
		flightState->ExitVelocity = FVector::ZeroVector;
		Engine.SetScriptedVelocity(FVector::ZeroVector);
		FinishAutopilot(TEXT("hazard"));
		bRouteReady = false;
		return false;
	}
	const double frameStartSeconds = RouteElapsedSeconds;
	const double frameDurationSeconds = FrameDeltaTime;
	TArray<FVector>& frameStartBodyPositions = flightState->FrameStartBodyPositions;
	TArray<FVector>& frameEndBodyPositions = flightState->FrameEndBodyPositions;
	frameStartBodyPositions.Reset();
	frameEndBodyPositions.Reset();
	frameStartBodyPositions.Reserve(MonitoredBodies.Num());
	frameEndBodyPositions.Reserve(MonitoredBodies.Num());
	for (const FMonitoredBody& body : MonitoredBodies)
	{
		frameStartBodyPositions.Add(body.PreviousLocation);
		const AActor* actor = body.Actor.Get();
		frameEndBodyPositions.Add(actor != nullptr ? actor->GetActorLocation() : body.PreviousLocation);
	}
	while (RouteElapsedSeconds < EndSeconds)
	{
		if (!segments.IsValidIndex(flightState->MotionSegmentIndex))
		{
			FinishAutopilot(TEXT("invalid"));
			return false;
		}
		const FScriptedMotionSegment& segment = segments[flightState->MotionSegmentIndex];
		const double segmentEndSeconds = segment.StartSeconds + segment.DurationSeconds;
		if (segmentEndSeconds <= RouteElapsedSeconds)
		{
			++flightState->MotionSegmentIndex;
			continue;
		}
		const double remainingFraction = FMath::Clamp(
			(segmentEndSeconds - RouteElapsedSeconds) / segment.DurationSeconds, 0.0, 1.0);
		const double currentFuelKg = GetScriptedFuelKg(segment.Fuel);
		const double requiredFuelKg = segment.bCurved
			? FMath::Max(0.0, segment.CurveRemainingFuelKg - currentFuelKg * (1.0 - remainingFraction))
			: currentFuelKg * remainingFraction;
		const double availableFuelKg = static_cast<double>(Resources.Resources.Fuel) +
			flightState->Telemetry.FuelRoundingKg;
		if (requiredFuelKg > 0.0 && requiredFuelKg > availableFuelKg +
			GetScriptedFuelToleranceKg(flightState->Telemetry.InitialFuelKg))
		{
			FinishAutopilot(TEXT("fuel"));
			return false;
		}
		const double intervalEndSeconds = segment.bCurved
			? FMath::Min(segmentEndSeconds, RouteElapsedSeconds + 0.025) : segmentEndSeconds;
		const double nextSeconds = FMath::Min(EndSeconds, intervalEndSeconds);
		const float stepSeconds = static_cast<float>(nextSeconds - RouteElapsedSeconds);
		FNavigationControl control;
		AddScriptedControl(control, segment.Fuel,
			(nextSeconds - RouteElapsedSeconds) / segment.DurationSeconds);
		FNavigationControl capped = control;
		LimitControlToFuel(capped, Resources.Resources.Fuel);
		ObserveAutopilotTelemetry(flightState->Telemetry, segment.Phase, 0, stepSeconds);
		const bool bCurrent = BroadcastAutopilotControl(Pawn, Resources, flightState->Telemetry,
			control, capped, stepSeconds, [this, flightState]()
			{
				return AutopilotState == flightState;
			});
		if (!bCurrent) return false;
		RouteElapsedSeconds = nextSeconds;
		UpdateScriptedPose(Pawn, Engine, Resources);
		if (AutopilotState != flightState) return false;
		const double frameFraction = (RouteElapsedSeconds - frameStartSeconds) / frameDurationSeconds;
		CheckRouteCollision(Pawn.GetActorLocation(), stepSeconds, frameFraction,
			frameStartBodyPositions, frameEndBodyPositions);
		PreviousPawnLocation = Pawn.GetActorLocation();
		bHasPreviousPawnLocation = true;
	}
	return AutopilotState == flightState;
}

void USpaceNavRouteComponent::UpdateScriptedPose(ASpaceNavPawn& Pawn,
	USpaceNavEngineComponent& Engine, USpaceNavResourceStoreComponent& Resources)
{
	const TSharedPtr<FRouteAutopilotState> poseState = AutopilotState;
	FRouteAutopilotState& flight = *poseState;
	const FRouteCandidate& candidate = *FindRouteCandidate(*RoutePool, SelectedCandidateIndex);
	const FScriptedPose pose = EvaluateScriptedMotion(candidate.MotionSegments,
		RouteElapsedSeconds, &flight.MotionSegmentIndex);
	const FRotator previousRotation = Pawn.GetActorRotation();
	const double stepSeconds = RouteElapsedSeconds - flight.Snapshot.ElapsedSeconds;
	Pawn.SetActorLocationAndRotation(pose.Position, pose.Orientation, false, nullptr,
		ETeleportType::TeleportPhysics);
	if (AutopilotState != poseState) return;
	Engine.SetScriptedVelocity(pose.Velocity);
	flight.ExitVelocity = pose.Velocity;
	NextRoutePointIndex = pose.NextPointIndex;
	FNavigationState state = SnapshotNavigationState(Pawn, Engine, Resources, RouteElapsedSeconds);
	if (stepSeconds > UE_SMALL_NUMBER)
	{
		const FRotator rotation = pose.Orientation.Rotator();
		state.YawAngularVelocity = FMath::FindDeltaAngleDegrees(previousRotation.Yaw,
			rotation.Yaw) / stepSeconds;
		state.PitchAngularVelocity = FMath::FindDeltaAngleDegrees(previousRotation.Pitch,
			rotation.Pitch) / stepSeconds;
	}
	flight.Snapshot = MakeAutopilotSnapshot(state, Resources.Resources.Fuel,
		GuidancePoints.Last(), 0.0);
	flight.Snapshot.SegmentIndex = flight.MotionSegmentIndex + 1;
	flight.Snapshot.SegmentCount = candidate.MotionSegments.Num();
}

void USpaceNavRouteComponent::FinishAutopilot(const TCHAR* Result)
{
	if (!AutopilotState.IsValid()) return;
	const TSharedPtr<FRouteAutopilotState> flight = AutopilotState;
	ASpaceNavPawn* pawn = Cast<ASpaceNavPawn>(GetOwner());
	USpaceNavEngineComponent* engine = flight->Engine.Get();
	USpaceNavResourceStoreComponent* resources = pawn != nullptr
		? pawn->FindComponentByClass<USpaceNavResourceStoreComponent>() : nullptr;
	if (pawn != nullptr && engine != nullptr && resources != nullptr && !GuidancePoints.IsEmpty())
	{
		const FNavigationState state = SnapshotNavigationState(*pawn, *engine, *resources,
			RouteElapsedSeconds);
		FAutopilotSnapshot snapshot = MakeAutopilotSnapshot(state, resources->Resources.Fuel,
			GuidancePoints.Last(), flight->Snapshot.CrossLineErrorUU);
		snapshot.SegmentIndex = flight->Snapshot.SegmentIndex;
		snapshot.SegmentCount = flight->Snapshot.SegmentCount;
		flight->Snapshot = snapshot;
	}
	if (engine != nullptr) engine->EndScriptedFlight(flight->ExitVelocity);
	FlushAutopilotTelemetry(flight->Telemetry, flight->Snapshot, bLogRoute, true);
	LogAutopilotSummary(flight->Telemetry, Result, bLogRoute);
	AutopilotState.Reset();
	NextRoutePointIndex = INDEX_NONE;
	SetComponentTickEnabled(false);
}

void USpaceNavRouteComponent::CaptureCollisionBodies()
{
	MonitoredBodies.Reset();
	UWorld* world = GetWorld();
	if (world == nullptr) return;
	for (TActorIterator<ASpaceNavCentralBody> bodyIterator(world); bodyIterator; ++bodyIterator)
	{
		ASpaceNavCentralBody* bodyActor = *bodyIterator;
		if (!FMath::IsFinite(bodyActor->RadiusMeters) || bodyActor->RadiusMeters <= 0.0) continue;
		FMonitoredBody& monitoredBody = MonitoredBodies.AddDefaulted_GetRef();
		monitoredBody.Actor = bodyActor;
		monitoredBody.PreviousLocation = bodyActor->GetActorLocation();
		monitoredBody.RadiusUU = bodyActor->RadiusMeters * UnrealUnitsPerMeter;
		monitoredBody.bStar = true;
	}
	for (TActorIterator<ASpaceNavPlanet> planetIterator(world); planetIterator; ++planetIterator)
	{
		ASpaceNavPlanet* planetActor = *planetIterator;
		if (!FMath::IsFinite(planetActor->RadiusMeters) || planetActor->RadiusMeters <= 0.0) continue;
		FMonitoredBody& monitoredBody = MonitoredBodies.AddDefaulted_GetRef();
		monitoredBody.Actor = planetActor;
		monitoredBody.PreviousLocation = planetActor->GetActorLocation();
		monitoredBody.RadiusUU = planetActor->RadiusMeters * UnrealUnitsPerMeter;
		monitoredBody.bStar = false;
	}
}

void USpaceNavRouteComponent::CheckRouteCollision(const FVector& PawnLocation, float DeltaTime,
	double FrameFraction, const TArray<FVector>& FrameStartBodyPositions,
	const TArray<FVector>& FrameEndBodyPositions)
{
	if (bImpactLogged || !bHasPreviousPawnLocation) return;
	const FVector pawnStep = PawnLocation - PreviousPawnLocation;
	const FRouteCandidate* curveCandidate = AutopilotState.IsValid() && RoutePool.IsValid()
		? FindRouteCandidate(*RoutePool, SelectedCandidateIndex) : nullptr;
	if (curveCandidate != nullptr &&
		(!curveCandidate->MotionSegments.IsValidIndex(AutopilotState->MotionSegmentIndex) ||
		 !curveCandidate->MotionSegments[AutopilotState->MotionSegmentIndex].bCurved))
		curveCandidate = nullptr;
	const double unitsPerKm = UnrealUnitsPerMeter * 1000.0;
	for (int32 bodyIndex = 0; bodyIndex < MonitoredBodies.Num(); ++bodyIndex)
	{
		FMonitoredBody& monitoredBody = MonitoredBodies[bodyIndex];
		AActor* bodyActor = monitoredBody.Actor.Get();
		if (bodyActor == nullptr) continue;
		const FVector currentBodyLocation = FMath::Lerp(FrameStartBodyPositions[bodyIndex],
			FrameEndBodyPositions[bodyIndex], FrameFraction);
		const FVector previousRelative = PreviousPawnLocation - monitoredBody.PreviousLocation;
		const FVector currentRelative = PawnLocation - currentBodyLocation;
		const FVector relativeStep = currentRelative - previousRelative;
		const double fraction = relativeStep.SizeSquared() > 0.0
			? FMath::Clamp(-FVector::DotProduct(previousRelative, relativeStep) /
				relativeStep.SizeSquared(), 0.0, 1.0) : 0.0;
		double centerDistanceUU = (previousRelative + relativeStep * fraction).Size();
		const FVector previousBodyLocation = monitoredBody.PreviousLocation;
		monitoredBody.PreviousLocation = currentBodyLocation;
		if (centerDistanceUU > monitoredBody.RadiusUU) continue;
		FVector impactLocation = PreviousPawnLocation + pawnStep * fraction;
		if (curveCandidate != nullptr)
		{
			impactLocation = EvaluateScriptedMotion(curveCandidate->MotionSegments,
				RouteElapsedSeconds - DeltaTime * (1.0 - fraction)).Position;
			centerDistanceUU = FVector::Dist(impactLocation,
				FMath::Lerp(previousBodyLocation, currentBodyLocation, fraction));
			if (centerDistanceUU > monitoredBody.RadiusUU) continue;
		}
		const double targetDistanceKm = GuidancePoints.IsEmpty() ? 0.0
			: FVector::Dist(impactLocation, GuidancePoints.Last()) / unitsPerKm;
		UE_LOG(LogSpaceNavRoute, Warning, TEXT("Impact %s"),
			monitoredBody.bStar ? TEXT("star") : TEXT("planet"));
		UE_LOG(LogSpaceNavRoute, Warning, TEXT("Body %s"), *bodyActor->GetName().Left(12));
		UE_LOG(LogSpaceNavRoute, Warning, TEXT("Time %.3gs"),
			RouteElapsedSeconds - DeltaTime * (1.0 - fraction));
		UE_LOG(LogSpaceNavRoute, Warning, TEXT("Center %.3gkm"), centerDistanceUU / unitsPerKm);
		UE_LOG(LogSpaceNavRoute, Warning, TEXT("Radius %.3gkm"), monitoredBody.RadiusUU / unitsPerKm);
		UE_LOG(LogSpaceNavRoute, Warning, TEXT("Goal %.3gkm"), targetDistanceKm);
		bImpactLogged = true;
		return;
	}
}

void USpaceNavRouteComponent::ResumeEngineAfterSearch()
{
	if (!bEnginePausedForSearch) return;
	if (USpaceNavEngineComponent* engine = PausedEngine.Get())
	{
		engine->SetComponentTickEnabled(bEngineTickWasEnabled);
	}
	bEnginePausedForSearch = false;
	PausedEngine.Reset();
	LogDisplay(TEXT("Ship released"));
}

void USpaceNavRouteComponent::LogDisplay(const TCHAR* Message) const
{
	if (!bLogRoute) return;
	UE_LOG(LogSpaceNavRoute, Display, TEXT("%s"), Message);
}

namespace
{
	FEngineSnapshot SnapshotEngine(const USpaceNavEngineComponent& Engine)
	{
		FEngineSnapshot snapshot;
		snapshot.ThrustAcceleration = Engine.ThrustAcceleration;
		snapshot.MaxSpeed = Engine.MaxSpeed;
		snapshot.TurnImpulsePerFuelUnit = Engine.TurnImpulsePerFuelUnit;
		snapshot.TurnDeceleration = Engine.TurnDeceleration;
		snapshot.MaxTurnSpeed = Engine.MaxTurnSpeed;
		return snapshot;
	}

	void AdvanceRoutePointIndex(const TArray<FVector>& Points, const FVector& Location, int32& NextPointIndex)
	{
		while (NextPointIndex < Points.Num() - 1)
		{
			const FVector segment = Points[NextPointIndex] - Points[NextPointIndex - 1];
			if (segment.SizeSquared() > 0.0 &&
				FVector::DotProduct(Location - Points[NextPointIndex], segment) < 0.0) break;
			++NextPointIndex;
		}
	}

	FVector ProjectOntoCurrentSegment(const TArray<FVector>& Points, const FVector& Location, int32 NextPointIndex)
	{
		const FVector start = Points[NextPointIndex - 1];
		const FVector segment = Points[NextPointIndex] - start;
		const double segmentLengthSquared = segment.SizeSquared();
		if (segmentLengthSquared <= 0.0) return start;
		const double fraction = FMath::Clamp(FVector::DotProduct(Location - start, segment) /
			segmentLengthSquared, 0.0, 1.0);
		return start + segment * fraction;
	}

	FVector FindLookAheadPoint(const TArray<FVector>& Points, const FVector& PathLocation,
		int32 NextPointIndex, double LookAheadDistance)
	{
		FVector currentLocation = PathLocation;
		for (int32 pointIndex = NextPointIndex; pointIndex < Points.Num(); ++pointIndex)
		{
			const FVector toNextPoint = Points[pointIndex] - currentLocation;
			const double segmentDistance = toNextPoint.Size();
			if (LookAheadDistance <= segmentDistance)
			{
				return currentLocation + toNextPoint * (LookAheadDistance / segmentDistance);
			}
			LookAheadDistance -= segmentDistance;
			currentLocation = Points[pointIndex];
		}
		return Points.Last();
	}

	float CalculateTurnFuel(double ErrorDegrees, float CurrentAngularVelocity,
		const FEngineSnapshot& Engine, float DeltaTime)
	{
		if (Engine.TurnImpulsePerFuelUnit <= 0.0f || DeltaTime <= 0.0f) return 0.0f;
		const double deceleration = FMath::Max(0.0, static_cast<double>(Engine.TurnDeceleration));
		const double desiredAngularVelocity = FMath::Sign(ErrorDegrees) *
			FMath::Min(static_cast<double>(Engine.MaxTurnSpeed),
				FMath::Sqrt(2.0 * deceleration * FMath::Abs(ErrorDegrees)));
		const double requestedFuel = (desiredAngularVelocity - CurrentAngularVelocity) /
			Engine.TurnImpulsePerFuelUnit;
		const double fuelLimit = MaxTurnFuelPerSecond * DeltaTime;
		return static_cast<float>(FMath::Clamp(requestedFuel, -fuelLimit, fuelLimit));
	}

	FNavigationControl CalculateControl(const TArray<FVector>& Points, FNavigationState& State,
		const FEngineSnapshot& Engine, float DeltaTime)
	{
		FNavigationControl control;
		if (Points.Num() < 2 || DeltaTime <= 0.0f) return control;
		AdvanceRoutePointIndex(Points, State.Position, State.NextPointIndex);
		const FVector pathLocation = ProjectOntoCurrentSegment(Points, State.Position, State.NextPointIndex);
		const FVector currentVelocity = State.EngineVelocity + State.GravityVelocity;
		const double currentSpeed = currentVelocity.Size();
		const double lookAheadDistance = FMath::Max(ArrivalDistance,
			currentSpeed * SteeringLookAheadSeconds);
		const FVector targetLocation = FindLookAheadPoint(Points, pathLocation,
			State.NextPointIndex, lookAheadDistance);
		const FVector toTarget = targetLocation - State.Position;
		if (toTarget.IsNearlyZero()) return control;
		const FVector localTarget = State.Orientation.UnrotateVector(toTarget);
		const double yawError = FMath::RadiansToDegrees(FMath::Atan2(localTarget.Y, localTarget.X));
		const double horizontalDistance = FMath::Sqrt(localTarget.X * localTarget.X +
			localTarget.Y * localTarget.Y);
		const double pitchError = FMath::RadiansToDegrees(FMath::Atan2(localTarget.Z, horizontalDistance));
		control.YawFuel = CalculateTurnFuel(yawError, State.YawAngularVelocity, Engine, DeltaTime);
		control.PitchFuel = CalculateTurnFuel(pitchError, State.PitchAngularVelocity, Engine, DeltaTime);
		if (Engine.MaxSpeed <= 0.0f || Engine.ThrustAcceleration <= 0.0f) return control;

		const FVector velocityError = toTarget.GetSafeNormal() * Engine.MaxSpeed - currentVelocity;
		const double lateralFuelLimit = MaxLateralFuelPerSecond * DeltaTime;
		const FVector rightVector = State.Orientation.RotateVector(FVector::RightVector);
		const FVector upVector = State.Orientation.RotateVector(FVector::UpVector);
		control.RightFuel = static_cast<float>(FMath::Clamp(
			FVector::DotProduct(velocityError, rightVector) / Engine.ThrustAcceleration,
			-lateralFuelLimit, lateralFuelLimit));
		control.UpFuel = static_cast<float>(FMath::Clamp(
			FVector::DotProduct(velocityError, upVector) / Engine.ThrustAcceleration,
			-lateralFuelLimit, lateralFuelLimit));
		if (FMath::Abs(yawError) > AlignmentToleranceDegrees ||
			FMath::Abs(pitchError) > AlignmentToleranceDegrees) return control;

		const FVector forwardVector = State.Orientation.RotateVector(FVector::ForwardVector);
		control.ForwardFuel = static_cast<float>(FMath::Clamp(
			FVector::DotProduct(velocityError, forwardVector) / Engine.ThrustAcceleration,
			0.0, MaxForwardFuelPerSecond * DeltaTime));
		return control;
	}

	FNavigationControl CalculateSparseControl(const TArray<FVector>& Points, FNavigationState& State,
		const FEngineSnapshot& Engine, float DeltaTime, bool& bLaunchComplete,
		double& NextCorrectionSeconds)
	{
		FNavigationControl control;
		if (Points.Num() < 2 || DeltaTime <= 0.0f) return control;
		AdvanceRoutePointIndex(Points, State.Position, State.NextPointIndex);
		const FVector currentVelocity = State.EngineVelocity + State.GravityVelocity;
		const double currentSpeed = currentVelocity.Size();
		if (!bLaunchComplete)
		{
			if (currentSpeed < Engine.MaxSpeed - 1.0 &&
				State.EngineVelocity.Size() < Engine.MaxSpeed - 1.0)
			{
				return CalculateControl(Points, State, Engine, DeltaTime);
			}
			bLaunchComplete = true;
		}
		if (currentSpeed <= 0.0) return CalculateControl(Points, State, Engine, DeltaTime);
		if (State.ElapsedSeconds < NextCorrectionSeconds) return control;
		const FVector pathLocation = ProjectOntoCurrentSegment(Points, State.Position,
			State.NextPointIndex);
		const FVector targetLocation = FindLookAheadPoint(Points, pathLocation,
			State.NextPointIndex, FMath::Max(ArrivalDistance,
				currentSpeed * SteeringLookAheadSeconds));
		const double targetDistance = FVector::Dist(State.Position, Points.Last());
		const double corridor = FMath::Max(ArrivalDistance * 0.5,
			FMath::Min(currentSpeed * 2.0, targetDistance * 0.1));
		const FVector velocityDirection = currentVelocity / currentSpeed;
		const double along = FMath::Max(0.0, FVector::DotProduct(
			targetLocation - State.Position, velocityDirection));
		const double projectedMiss = FVector::Dist(
			State.Position + velocityDirection * along, targetLocation);
		if (projectedMiss <= corridor &&
			FVector::Dist(State.Position, pathLocation) <= corridor) return control;
		control = CalculateControl(Points, State, Engine, DeltaTime);
		if (currentSpeed >= Engine.MaxSpeed) control.ForwardFuel = 0.0f;
		NextCorrectionSeconds = State.ElapsedSeconds + FMath::Min(2.0,
			targetDistance / currentSpeed * 0.25);
		return control;
	}

	void LimitControlToFuel(FNavigationControl& Control, double AvailableFuelKg)
	{
		double remainingFuelKg = FMath::Max(0.0, AvailableFuelKg);
		const auto limitSigned = [&remainingFuelKg](float& requestedFuel)
		{
			const double grantedFuel = FMath::Min(
				static_cast<double>(FMath::Abs(requestedFuel)), remainingFuelKg);
			requestedFuel = static_cast<float>(FMath::Sign(requestedFuel) * grantedFuel);
			remainingFuelKg -= grantedFuel;
		};
		limitSigned(Control.YawFuel);
		limitSigned(Control.PitchFuel);
		limitSigned(Control.RightFuel);
		limitSigned(Control.UpFuel);
		Control.ForwardFuel = static_cast<float>(FMath::Min(
			static_cast<double>(FMath::Max(0.0f, Control.ForwardFuel)), remainingFuelKg));
	}

	double CalculatePredictionReserveUU(double RadiusUU, double RelativeSpeedUUPerSecond,
		double StepSeconds)
	{
		return FMath::Max(RadiusUU * MinimumBodyReserveFraction,
			RelativeSpeedUUPerSecond * StepSeconds);
	}

	double CalculateBodyPredictionReserveUU(const FPlannerBody& Body, double RelativeSpeedUUPerSecond,
		double StepSeconds)
	{
		return FMath::Max(CalculatePredictionReserveUU(Body.RadiusUU, RelativeSpeedUUPerSecond, StepSeconds),
			FMath::Max(0.0, Body.CriticalRadiusUU - Body.RadiusUU));
	}

	double CalculateShipToPlanetSpeedRisk(const FPlannerContext& Context, const FPlannerBody& Body,
		double ShipSpeedUUPerSecond)
	{
		if (!Body.bPlanet || Context.ShipToPlanetSpeedFactor <= 0.0) return 0.0;
		const double planetSpeedUUPerSecond = FVector::CrossProduct(Body.Orbit.Axis.GetSafeNormal(),
			Body.Orbit.InitialOffset).Size() * FMath::Abs(Body.Orbit.AngularSpeedRadiansPerSecond);
		const double desiredSpeedUUPerSecond = planetSpeedUUPerSecond * Context.ShipToPlanetSpeedFactor;
		return desiredSpeedUUPerSecond > 0.0
			? FMath::Clamp(1.0 - ShipSpeedUUPerSecond / desiredSpeedUUPerSecond, 0.0, 1.0) : 0.0;
	}

	double CalculateTurnReserveUU(double RadiusUU, double RelativeSpeedUUPerSecond,
		const FEngineSnapshot& Engine)
	{
		const double lateralAcceleration = FMath::Max(1.0,
			static_cast<double>(Engine.ThrustAcceleration) * MaxLateralFuelPerSecond);
		return FMath::Max(RadiusUU * MinimumBodyReserveFraction,
			FMath::Square(RelativeSpeedUUPerSecond) / lateralAcceleration);
	}

	FPredictedStateSample InterpolatePredictedState(const TArray<FPredictedStateSample>& Samples,
		double TimeSeconds, int32& NextSampleIndex)
	{
		NextSampleIndex = FMath::Clamp(NextSampleIndex, 1, Samples.Num() - 1);
		while (NextSampleIndex < Samples.Num() - 1 &&
			Samples[NextSampleIndex].TimeSeconds < TimeSeconds) ++NextSampleIndex;
		while (NextSampleIndex > 1 &&
			Samples[NextSampleIndex - 1].TimeSeconds > TimeSeconds) --NextSampleIndex;
		const FPredictedStateSample& previous = Samples[NextSampleIndex - 1];
		const FPredictedStateSample& next = Samples[NextSampleIndex];
		const double alpha = FMath::Clamp((TimeSeconds - previous.TimeSeconds) /
			FMath::Max(0.001, next.TimeSeconds - previous.TimeSeconds), 0.0, 1.0);
		FPredictedStateSample interpolated;
		interpolated.TimeSeconds = FMath::Lerp(previous.TimeSeconds, next.TimeSeconds, alpha);
		interpolated.Position = FMath::Lerp(previous.Position, next.Position, alpha);
		interpolated.Velocity = FMath::Lerp(previous.Velocity, next.Velocity, alpha);
		return interpolated;
	}

	FNavigationControl CalculateCorrectionControl(const FVector& DesiredVelocityChange,
		const FQuat& Orientation, const USpaceNavEngineComponent& Engine, float DeltaTime)
	{
		FNavigationControl control;
		if (Engine.ThrustAcceleration <= 0.0f || DeltaTime <= 0.0f) return control;
		const FVector localChange = Orientation.UnrotateVector(DesiredVelocityChange);
		const double lateralLimit = MaxLateralFuelPerSecond * DeltaTime;
		control.RightFuel = static_cast<float>(FMath::Clamp(
			localChange.Y / Engine.ThrustAcceleration, -lateralLimit, lateralLimit));
		control.UpFuel = static_cast<float>(FMath::Clamp(
			localChange.Z / Engine.ThrustAcceleration, -lateralLimit, lateralLimit));
		if (localChange.X >= 0.0)
		{
			control.ForwardFuel = static_cast<float>(FMath::Clamp(
				localChange.X / Engine.ThrustAcceleration, 0.0,
				MaxForwardFuelPerSecond * DeltaTime));
			return control;
		}
		const double yawError = FMath::RadiansToDegrees(FMath::Atan2(localChange.Y,
			localChange.X));
		const double horizontalDistance = FMath::Sqrt(
			localChange.X * localChange.X + localChange.Y * localChange.Y);
		const double pitchError = FMath::RadiansToDegrees(FMath::Atan2(localChange.Z,
			horizontalDistance));
		const FEngineSnapshot snapshot = SnapshotEngine(Engine);
		control.YawFuel = CalculateTurnFuel(yawError, Engine.GetYawAngularVelocity(),
			snapshot, DeltaTime);
		control.PitchFuel = CalculateTurnFuel(pitchError, Engine.GetPitchAngularVelocity(),
			snapshot, DeltaTime);
		return control;
	}

	void BroadcastControl(ASpaceNavPawn& Pawn, const FNavigationControl& Control)
	{
		if (Control.YawFuel > 0.0f) Pawn.OnTurnYawRight.Broadcast(Control.YawFuel);
		else if (Control.YawFuel < 0.0f) Pawn.OnTurnYawLeft.Broadcast(-Control.YawFuel);
		if (Control.PitchFuel > 0.0f) Pawn.OnTurnPitchUp.Broadcast(Control.PitchFuel);
		else if (Control.PitchFuel < 0.0f) Pawn.OnTurnPitchDown.Broadcast(-Control.PitchFuel);
		if (Control.RightFuel > 0.0f) Pawn.OnMoveRight.Broadcast(Control.RightFuel);
		else if (Control.RightFuel < 0.0f) Pawn.OnMoveLeft.Broadcast(-Control.RightFuel);
		if (Control.UpFuel > 0.0f) Pawn.OnMoveUp.Broadcast(Control.UpFuel);
		else if (Control.UpFuel < 0.0f) Pawn.OnMoveDown.Broadcast(-Control.UpFuel);
		if (Control.ForwardFuel > 0.0f) Pawn.OnMoveForward.Broadcast(Control.ForwardFuel);
	}

	FVector FPlannerBody::LocationAfter(double SecondsAhead) const
	{
		return bPlanet ? Orbit.LocationAfter(SecondsAhead) : FixedCenter;
	}

	bool BuildPlannerContext(const ASpaceNavPawn& Pawn, const USpaceNavEngineComponent& Engine,
		const USpaceNavResourceStoreComponent& Resources, const FVector& Target,
		double PlanetSafetyBufferFraction, FPlannerContext& OutContext)
	{
		UWorld* world = Pawn.GetWorld();
		if (world == nullptr)
		{
			UE_LOG(LogSpaceNavRoute, Error, TEXT("Failed: physics world missing"));
			return false;
		}
		if (!FMath::IsFinite(PlanetSafetyBufferFraction) || PlanetSafetyBufferFraction < 0.0)
		{
			UE_LOG(LogSpaceNavRoute, Error, TEXT("Failed: planet safety buffer invalid value=%.6g"),
				PlanetSafetyBufferFraction);
			return false;
		}
		bool bFoundGeneratorData = false;
		for (TActorIterator<ASpaceNavSystemGenerator> generatorIterator(world);
			generatorIterator; ++generatorIterator)
		{
			if (generatorIterator->GeneratorData == nullptr) continue;
			bFoundGeneratorData = true;
			OutContext.GravityCoefficient = generatorIterator->GeneratorData->GravityCoefficient;
			break;
		}
		if (!bFoundGeneratorData)
		{
			UE_LOG(LogSpaceNavRoute, Error, TEXT("Failed: physics generator data missing"));
			return false;
		}
		if (!FMath::IsFinite(OutContext.GravityCoefficient) || OutContext.GravityCoefficient <= 0.0)
		{
			UE_LOG(LogSpaceNavRoute, Error, TEXT("Failed: gravity coefficient invalid value=%.6g"),
				OutContext.GravityCoefficient);
			return false;
		}
		OutContext.Engine = SnapshotEngine(Engine);
		OutContext.Target = Target;
		OutContext.InitialFuelKg = Resources.Resources.Fuel;
		if (!FMath::IsFinite(OutContext.InitialFuelKg) || OutContext.InitialFuelKg < 0.0)
		{
			UE_LOG(LogSpaceNavRoute, Error, TEXT("Failed: physics fuel invalid value=%.6g"),
				OutContext.InitialFuelKg);
			return false;
		}
		OutContext.InitialState.Position = Pawn.GetActorLocation();
		OutContext.InitialState.Orientation = Pawn.GetActorQuat();
		OutContext.InitialState.EngineVelocity = Engine.GetLinearVelocity();
		OutContext.InitialState.GravityVelocity = Engine.GetGravityVelocity();
		OutContext.InitialState.YawAngularVelocity = Engine.GetYawAngularVelocity();
		OutContext.InitialState.PitchAngularVelocity = Engine.GetPitchAngularVelocity();
		OutContext.InitialState.FuelKg = OutContext.InitialFuelKg;

		for (TActorIterator<ASpaceNavCentralBody> bodyIterator(world); bodyIterator; ++bodyIterator)
		{
			FPlannerBody body;
			body.Actor = *bodyIterator;
			body.FixedCenter = bodyIterator->GetActorLocation();
			body.MassTonnes = bodyIterator->MassTonnes;
			body.RadiusUU = bodyIterator->RadiusMeters * UnrealUnitsPerMeter;
			body.AvoidRadiusUU = body.RadiusUU;
			body.CriticalRadiusUU = body.RadiusUU * (1.0 + OutContext.CriticalBodyClearanceRadiusFraction);
			if (!FMath::IsFinite(body.RadiusUU) || !FMath::IsFinite(body.MassTonnes) ||
				body.RadiusUU <= 0.0 || body.MassTonnes <= 0.0)
			{
				UE_LOG(LogSpaceNavRoute, Error,
					TEXT("Failed: central body data invalid actor=%s mass=%.6g radius_uu=%.6g"),
					*bodyIterator->GetName(), body.MassTonnes, body.RadiusUU);
				return false;
			}
			if (FVector::Dist(Pawn.GetActorLocation(), body.FixedCenter) < body.CriticalRadiusUU)
			{
				UE_LOG(LogSpaceNavRoute, Error, TEXT("Start critical body %d"), OutContext.Bodies.Num());
				return false;
			}
			OutContext.Bodies.Add(body);
		}
		for (TActorIterator<ASpaceNavPlanet> planetIterator(world); planetIterator; ++planetIterator)
		{
			FPlannerBody body;
			body.Actor = *planetIterator;
			body.bPlanet = true;
			body.Orbit = planetIterator->GetOrbitSnapshot();
			body.MassTonnes = planetIterator->MassTonnes;
			body.RadiusUU = planetIterator->RadiusMeters * UnrealUnitsPerMeter;
			body.AvoidRadiusUU = body.RadiusUU * (1.0 + PlanetSafetyBufferFraction);
			body.CriticalRadiusUU = body.RadiusUU * (1.0 + OutContext.CriticalBodyClearanceRadiusFraction);
			if (!FMath::IsFinite(body.RadiusUU) || !FMath::IsFinite(body.AvoidRadiusUU) ||
				!FMath::IsFinite(body.MassTonnes) || body.RadiusUU <= 0.0 ||
				body.AvoidRadiusUU <= 0.0 || body.MassTonnes <= 0.0)
			{
				UE_LOG(LogSpaceNavRoute, Error,
					TEXT("Failed: planet data invalid actor=%s mass=%.6g radius_uu=%.6g avoid_uu=%.6g"),
					*planetIterator->GetName(), body.MassTonnes, body.RadiusUU, body.AvoidRadiusUU);
				return false;
			}
			const double startDistance = FVector::Dist(Pawn.GetActorLocation(), body.LocationAfter(0.0));
			if (startDistance < body.CriticalRadiusUU)
			{
				UE_LOG(LogSpaceNavRoute, Error, TEXT("Start critical body %d"), OutContext.Bodies.Num());
				return false;
			}
			OutContext.Bodies.Add(body);
		}
		if (OutContext.Bodies.IsEmpty())
		{
			UE_LOG(LogSpaceNavRoute, Error, TEXT("Failed: physics bodies missing"));
			return false;
		}
		return true;
	}

	void CachePredictedBodyPositions(const FPlannerContext& Context, double ElapsedSeconds,
		TArray<FVector>& OutPositions)
	{
		OutPositions.SetNumUninitialized(Context.Bodies.Num());
		for (int32 bodyIndex = 0; bodyIndex < Context.Bodies.Num(); ++bodyIndex)
		{
			OutPositions[bodyIndex] = Context.Bodies[bodyIndex].LocationAfter(ElapsedSeconds);
		}
	}

	double GetPredictedTurnTime(const FVector& DesiredDirection, const FNavigationState& State,
		const FEngineSnapshot& Engine)
	{
		if (DesiredDirection.IsNearlyZero()) return 0.0;
		const FVector forward = State.Orientation.RotateVector(FVector::ForwardVector);
		const double angle = FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(
			FVector::DotProduct(forward, DesiredDirection.GetSafeNormal()), -1.0, 1.0)));
		const double turnSpeed = FMath::Max(1.0, static_cast<double>(Engine.MaxTurnSpeed));
		const double turnAcceleration = FMath::Max(1.0,
			static_cast<double>(Engine.TurnImpulsePerFuelUnit) * MaxTurnFuelPerSecond);
		return angle / turnSpeed + turnSpeed / turnAcceleration +
			turnSpeed / FMath::Max(1.0, static_cast<double>(Engine.TurnDeceleration));
	}

	double GetPredictedBrakingDistance(const FNavigationState& State, const FEngineSnapshot& Engine)
	{
		const FVector velocity = State.EngineVelocity + State.GravityVelocity;
		const double speed = velocity.Size();
		const double acceleration = FMath::Max(1.0,
			static_cast<double>(Engine.ThrustAcceleration) * MaxForwardFuelPerSecond);
		return ArrivalDistance + speed * GetPredictedTurnTime(-velocity, State, Engine) +
			speed * speed / (2.0 * acceleration);
	}

	FNavigationControl CalculatePredictedAlignment(const FVector& DesiredDirection,
		const FNavigationState& State, const FEngineSnapshot& Engine, float DeltaTime,
		bool& bOutAligned)
	{
		FNavigationControl control;
		bOutAligned = false;
		if (DesiredDirection.IsNearlyZero() || DeltaTime <= 0.0f) return control;
		const FVector localDirection = State.Orientation.UnrotateVector(DesiredDirection);
		const double yawError = FMath::RadiansToDegrees(FMath::Atan2(localDirection.Y, localDirection.X));
		const double horizontal = FMath::Sqrt(localDirection.X * localDirection.X +
			localDirection.Y * localDirection.Y);
		const double pitchError = FMath::RadiansToDegrees(FMath::Atan2(localDirection.Z, horizontal));
		const bool bHeadingReady = FMath::Abs(yawError) <= AlignmentToleranceDegrees &&
			FMath::Abs(pitchError) <= AlignmentToleranceDegrees;
		control.YawFuel = CalculateTurnFuel(bHeadingReady ? 0.0 : yawError,
			State.YawAngularVelocity, Engine, DeltaTime);
		control.PitchFuel = CalculateTurnFuel(bHeadingReady ? 0.0 : pitchError,
			State.PitchAngularVelocity, Engine, DeltaTime);
		bOutAligned = bHeadingReady && FMath::IsNearlyZero(State.YawAngularVelocity) &&
			FMath::IsNearlyZero(State.PitchAngularVelocity);
		return control;
	}

	FVector ResolvePredictedTargetVelocity(const FVector& NominalVelocity,
		const FVector& CurrentVelocity, const FFlightFeedback& Feedback, bool bAllowTracking)
	{
		FVector targetVelocity = bAllowTracking && Feedback.bTracking
			? Feedback.TargetVelocity : NominalVelocity;
		if (!Feedback.bAvoidance) return targetVelocity;
		const FVector avoidanceDirection = Feedback.AvoidanceDirection.GetSafeNormal();
		if (avoidanceDirection.IsNearlyZero()) return targetVelocity;
		const double requestedChange = FVector::DotProduct(targetVelocity - CurrentVelocity,
			avoidanceDirection);
		targetVelocity += avoidanceDirection * FMath::Max(0.0,
			Feedback.MinimumAvoidanceSpeed - requestedChange);
		return targetVelocity;
	}

	FNavigationControl ConvertPredictedVelocityChange(const FVector& VelocityChange,
		const FVector& HeadingDirection, const FNavigationState& State,
		const FEngineSnapshot& Engine, float DeltaTime)
	{
		bool bAligned = false;
		FNavigationControl control = CalculatePredictedAlignment(HeadingDirection,
			State, Engine, DeltaTime, bAligned);
		if (Engine.ThrustAcceleration <= 0.0f || DeltaTime <= 0.0f) return control;
		const FVector localChange = State.Orientation.UnrotateVector(VelocityChange);
		const double lateralLimit = MaxLateralFuelPerSecond * DeltaTime;
		control.RightFuel = static_cast<float>(FMath::Clamp(localChange.Y / Engine.ThrustAcceleration,
			-lateralLimit, lateralLimit));
		control.UpFuel = static_cast<float>(FMath::Clamp(localChange.Z / Engine.ThrustAcceleration,
			-lateralLimit, lateralLimit));
		if (bAligned) control.ForwardFuel = static_cast<float>(FMath::Clamp(
			localChange.X / Engine.ThrustAcceleration, 0.0, MaxForwardFuelPerSecond * DeltaTime));
		return control;
	}

	void InitializePredictedSegment(const TArray<FVector>& Points,
		const FNavigationState& State, const FEngineSnapshot& Engine,
		double CruiseSpeedFraction, FPhasedFlightState& Flight)
	{
		const FVector toTarget = Points[Flight.NextPointIndex] - State.Position;
		Flight.SegmentDirection = toTarget.GetSafeNormal();
		Flight.SegmentPointIndex = Flight.NextPointIndex;
		const double acceleration = static_cast<double>(Engine.ThrustAcceleration) * MaxForwardFuelPerSecond;
		const double usableDistance = FMath::Max(ArrivalDistance, toTarget.Size() - ArrivalDistance);
		double attainableSpeed = FMath::Sqrt(acceleration * usableDistance);
		if (Flight.NextPointIndex < Points.Num() - 1 && !Flight.bCornerBraked)
		{
			const double turnTime = GetPredictedTurnTime(-toTarget, State, Engine);
			const double turnSpeedLoss = acceleration * turnTime * 0.5;
			attainableSpeed = FMath::Sqrt(turnSpeedLoss * turnSpeedLoss +
				acceleration * usableDistance * 0.5) - turnSpeedLoss;
		}
		Flight.CruiseSpeedUU = FMath::Min(static_cast<double>(Engine.MaxSpeed),
			FMath::Max(1.0, attainableSpeed)) * CruiseSpeedFraction;
		Flight.NextCorrectionSeconds = State.ElapsedSeconds;
	}

	void AdvancePredictedCorner(const TArray<FVector>& Points, FNavigationState& State,
		double StoppedSpeed, FPhasedFlightState& Flight)
	{
		const FVector previousPosition = Flight.bHasPreviousPosition
			? Flight.PreviousPosition : State.Position;
		const FVector travel = State.Position - previousPosition;
		const double travelSquared = travel.SizeSquared();
		while (Flight.NextPointIndex < Points.Num() - 1)
		{
			const FVector corner = Points[Flight.NextPointIndex];
			const double fraction = travelSquared > 0.0
				? FMath::Clamp(FVector::DotProduct(corner - previousPosition, travel) / travelSquared,
					0.0, 1.0) : 0.0;
			const bool bSweptCorner = FVector::DistSquared(previousPosition + travel * fraction, corner) <=
				FMath::Square(ArrivalDistance);
			const FVector toCorner = corner - State.Position;
			const double remaining = FVector::DotProduct(toCorner, Flight.SegmentDirection);
			const bool bPassedCorner = Flight.SegmentPointIndex == Flight.NextPointIndex &&
				remaining <= 0.0 && (toCorner - Flight.SegmentDirection * remaining).SizeSquared() <=
				FMath::Square(ArrivalDistance);
			if (!bSweptCorner && !bPassedCorner) break;
			Flight.BrakingPointIndex = Flight.NextPointIndex;
			++Flight.NextPointIndex;
			Flight.bCornerBraked = false;
			Flight.SegmentPointIndex = INDEX_NONE;
			Flight.Phase = (State.EngineVelocity + State.GravityVelocity).Size() > StoppedSpeed
				? EFlightPhase::Brake : EFlightPhase::Redirect;
		}
		Flight.PreviousPosition = State.Position;
		Flight.bHasPreviousPosition = true;
		State.NextPointIndex = Flight.NextPointIndex;
	}

	FNavigationControl CalculatePhasedControl(const TArray<FVector>& Points, FNavigationState& State,
		const FEngineSnapshot& Engine, double CruiseSpeedFraction, float DeltaTime,
		FPhasedFlightState& Flight, const FFlightFeedback& Feedback)
	{
		FNavigationControl control;
		if (Points.Num() < 2 || Engine.ThrustAcceleration <= 0.0f || DeltaTime <= 0.0f) return control;
		Flight.NextPointIndex = FMath::Clamp(Flight.NextPointIndex, 1, Points.Num() - 1);
		const FVector velocity = State.EngineVelocity + State.GravityVelocity;
		const double speed = velocity.Size();
		const double gravityImpulse = FVector::Dist(State.GravityVelocity, Flight.PreviousGravityVelocity);
		Flight.PreviousGravityVelocity = State.GravityVelocity;
		const double stoppedSpeed = FMath::Max(1.0, gravityImpulse + KINDA_SMALL_NUMBER);
		AdvancePredictedCorner(Points, State, stoppedSpeed, Flight);
		if (Flight.Phase == EFlightPhase::Brake && speed <= stoppedSpeed)
		{
			Flight.bCornerBraked = Flight.BrakingPointIndex == Flight.NextPointIndex;
			Flight.Phase = EFlightPhase::Redirect;
		}
		if (Flight.Phase == EFlightPhase::Redirect)
		{
			Flight.SegmentPointIndex = INDEX_NONE;
			Flight.Phase = EFlightPhase::Align;
		}
		if (Flight.SegmentPointIndex != Flight.NextPointIndex)
		{
			InitializePredictedSegment(Points, State, Engine, CruiseSpeedFraction, Flight);
		}
		const FVector toTarget = Points[Flight.NextPointIndex] - State.Position;
		const double targetDistance = toTarget.Size();
		const double remainingAlongCourse = FVector::DotProduct(toTarget, Flight.SegmentDirection);
		const bool bIntermediateCorner = Flight.NextPointIndex < Points.Num() - 1;
		if ((Flight.Phase == EFlightPhase::Launch || Flight.Phase == EFlightPhase::Coast) &&
			speed > stoppedSpeed && (remainingAlongCourse < -ArrivalDistance ||
			(bIntermediateCorner && !Flight.bCornerBraked &&
				targetDistance <= GetPredictedBrakingDistance(State, Engine))))
		{
			Flight.BrakingPointIndex = Flight.NextPointIndex;
			Flight.Phase = EFlightPhase::Brake;
		}
		if (Flight.Phase == EFlightPhase::Brake)
		{
			const FVector brakeVelocity = ResolvePredictedTargetVelocity(FVector::ZeroVector,
				velocity, Feedback, false);
			return ConvertPredictedVelocityChange(brakeVelocity - velocity, -velocity,
				State, Engine, DeltaTime);
		}
		if (Flight.Phase == EFlightPhase::Align)
		{
			bool bAligned = false;
			control = CalculatePredictedAlignment(Flight.SegmentDirection, State, Engine, DeltaTime, bAligned);
			if (!bAligned)
			{
				if (Feedback.bAvoidance)
				{
					const FVector avoidanceVelocity = ResolvePredictedTargetVelocity(velocity,
						velocity, Feedback, false);
					return ConvertPredictedVelocityChange(avoidanceVelocity - velocity,
						Flight.SegmentDirection, State, Engine, DeltaTime);
				}
				return control;
			}
			Flight.Phase = EFlightPhase::Launch;
		}
		FVector nominalVelocity = Flight.SegmentDirection * Flight.CruiseSpeedUU;
		if (!Feedback.bTracking)
		{
			const FVector pathPosition = ProjectOntoCurrentSegment(Points, State.Position, Flight.NextPointIndex);
			const FVector crossTrackError = pathPosition - State.Position;
			nominalVelocity += (crossTrackError - Flight.SegmentDirection *
				FVector::DotProduct(crossTrackError, Flight.SegmentDirection)) / CorrectionWindowSeconds;
		}
		const FVector targetVelocity = ResolvePredictedTargetVelocity(nominalVelocity,
			velocity, Feedback, true);
		const FVector velocityChange = targetVelocity - velocity;
		const FVector localChange = State.Orientation.UnrotateVector(velocityChange);
		const double controllableError = FMath::Sqrt(FMath::Square(FMath::Max(0.0, localChange.X)) +
			FMath::Square(localChange.Y) + FMath::Square(localChange.Z));
		const double remainingSeconds = targetDistance / FMath::Max(1.0, FMath::Max(speed, Flight.CruiseSpeedUU));
		const double velocityTolerance = FMath::Max(static_cast<double>(KINDA_SMALL_NUMBER),
			FMath::Min(Flight.CruiseSpeedUU * AlignmentToleranceDegrees / 180.0,
				FMath::Max(stoppedSpeed, ArrivalDistance / FMath::Max(CorrectionWindowSeconds, remainingSeconds))));
		if (Flight.Phase == EFlightPhase::Coast && !Feedback.bTracking && !Feedback.bAvoidance &&
			State.ElapsedSeconds < Flight.NextCorrectionSeconds) return FNavigationControl();
		if (speed > 0.0 && controllableError <= velocityTolerance && !Feedback.bAvoidance)
		{
			Flight.Phase = EFlightPhase::Coast;
			Flight.NextCorrectionSeconds = State.ElapsedSeconds +
				FMath::Min(CorrectionWindowSeconds, FMath::Max(static_cast<double>(DeltaTime), remainingSeconds * 0.25));
			return FNavigationControl();
		}
		Flight.Phase = EFlightPhase::Launch;
		return ConvertPredictedVelocityChange(velocityChange, Flight.SegmentDirection, State, Engine, DeltaTime);
	}

	FNavigationControl CalculatePredictedControl(const TArray<FVector>& Points,
		FNavigationState& State, const FEngineSnapshot& Engine, EPlanFamily Family,
		double CruiseSpeedFraction, float DeltaTime, FPhasedFlightState& Flight,
		const FFlightFeedback& Feedback)
	{
		if (Family == EPlanFamily::SparseBurns)
			return CalculatePhasedControl(Points, State, Engine, CruiseSpeedFraction, DeltaTime, Flight, Feedback);
		FNavigationControl control;
		if (Points.Num() < 2 || DeltaTime <= 0.0f) return control;
		AdvanceRoutePointIndex(Points, State.Position, State.NextPointIndex);
		Flight.NextPointIndex = State.NextPointIndex;
		Flight.CruiseSpeedUU = Engine.MaxSpeed * CruiseSpeedFraction;
		const FVector currentVelocity = State.EngineVelocity + State.GravityVelocity;
		const FVector pathPosition = ProjectOntoCurrentSegment(Points, State.Position, State.NextPointIndex);
		const double lookAhead = FMath::Max(ArrivalDistance, currentVelocity.Size() * SteeringLookAheadSeconds);
		const FVector targetPosition = FindLookAheadPoint(Points, pathPosition, State.NextPointIndex, lookAhead);
		const FVector nominalDirection = (targetPosition - State.Position).GetSafeNormal();
		const FVector nominalVelocity = nominalDirection * Engine.MaxSpeed * CruiseSpeedFraction;
		const FVector trackingVelocity = Feedback.bTracking ? Feedback.TargetVelocity : nominalVelocity;
		const FVector heading = trackingVelocity.IsNearlyZero() ? nominalDirection : trackingVelocity.GetSafeNormal();
		const FVector targetVelocity = ResolvePredictedTargetVelocity(nominalVelocity, currentVelocity, Feedback, true);
		Flight.SegmentDirection = heading;
		control = ConvertPredictedVelocityChange(targetVelocity - currentVelocity, heading, State, Engine, DeltaTime);
		bool bAligned = false;
		CalculatePredictedAlignment(heading, State, Engine, DeltaTime, bAligned);
		const bool bBurn = control.ForwardFuel > 0.0f || control.RightFuel != 0.0f ||
			control.UpFuel != 0.0f || control.YawFuel != 0.0f || control.PitchFuel != 0.0f;
		Flight.Phase = !bAligned ? EFlightPhase::Align : (bBurn ? EFlightPhase::Launch : EFlightPhase::Coast);
		return control;
	}

	double GetPredictionStepSeconds(const FNavigationState& State, const FPlannerContext& Context,
		const TArray<FVector>& CurrentBodyPositions)
	{
		double minimumClearanceUU = TNumericLimits<double>::Max();
		for (int32 bodyIndex = 0; bodyIndex < Context.Bodies.Num(); ++bodyIndex)
		{
			const double clearance = FVector::Dist(State.Position, CurrentBodyPositions[bodyIndex]) -
				Context.Bodies[bodyIndex].AvoidRadiusUU;
			minimumClearanceUU = FMath::Min(minimumClearanceUU, FMath::Max(0.0, clearance));
		}
		const double speedUU = (State.EngineVelocity + State.GravityVelocity).Size();
		const double clearanceStep = minimumClearanceUU /
			FMath::Max(100.0, speedUU + Context.Engine.MaxSpeed * 0.1) * 0.2;
		const double targetDistance = FVector::Dist(State.Position, Context.Target);
		const double targetStep = targetDistance / FMath::Max(100.0, speedUU) * 0.2;
		const double angularSpeed = FMath::Max(FMath::Abs(static_cast<double>(State.YawAngularVelocity)),
			FMath::Abs(static_cast<double>(State.PitchAngularVelocity)));
		const double angularStep = angularSpeed > 0.0 ? AlignmentToleranceDegrees / angularSpeed : 0.25;
		return FMath::Clamp(FMath::Min(FMath::Min(clearanceStep, targetStep), angularStep), 0.025, 0.25);
	}

	void AdvanceSimulatedState(FNavigationState& State, const FNavigationControl& Control,
		const FPlannerContext& Context, const TArray<FVector>& CurrentBodyPositions, float DeltaTime)
	{
		const FEngineSnapshot& engine = Context.Engine;
		State.FuelKg = FMath::Max(0.0, State.FuelKg -
			FMath::Abs(Control.YawFuel) - FMath::Abs(Control.PitchFuel) -
			FMath::Abs(Control.RightFuel) - FMath::Abs(Control.UpFuel) - Control.ForwardFuel);
		if (Control.RightFuel != 0.0f)
		{
			State.EngineVelocity += State.Orientation.RotateVector(FVector::RightVector) *
				(engine.ThrustAcceleration * Control.RightFuel);
			State.EngineVelocity = State.EngineVelocity.GetClampedToMaxSize(engine.MaxSpeed);
		}
		if (Control.UpFuel != 0.0f)
		{
			State.EngineVelocity += State.Orientation.RotateVector(FVector::UpVector) *
				(engine.ThrustAcceleration * Control.UpFuel);
			State.EngineVelocity = State.EngineVelocity.GetClampedToMaxSize(engine.MaxSpeed);
		}
		if (Control.ForwardFuel > 0.0f)
		{
			State.EngineVelocity += State.Orientation.RotateVector(FVector::ForwardVector) *
				(engine.ThrustAcceleration * Control.ForwardFuel);
			State.EngineVelocity = State.EngineVelocity.GetClampedToMaxSize(engine.MaxSpeed);
		}
		State.YawAngularVelocity = FMath::Clamp(State.YawAngularVelocity +
			engine.TurnImpulsePerFuelUnit * Control.YawFuel, -engine.MaxTurnSpeed, engine.MaxTurnSpeed);
		State.PitchAngularVelocity = FMath::Clamp(State.PitchAngularVelocity +
			engine.TurnImpulsePerFuelUnit * Control.PitchFuel, -engine.MaxTurnSpeed, engine.MaxTurnSpeed);

		FVector gravityAcceleration = FVector::ZeroVector;
		for (int32 bodyIndex = 0; bodyIndex < Context.Bodies.Num(); ++bodyIndex)
		{
			const FPlannerBody& body = Context.Bodies[bodyIndex];
			gravityAcceleration += USpaceNavEngineComponent::CalculateBodyGravityAcceleration(
				State.Position, CurrentBodyPositions[bodyIndex], body.MassTonnes,
				body.RadiusUU / UnrealUnitsPerMeter, Context.GravityCoefficient);
		}
		const FVector previousGravityVelocity = State.GravityVelocity;
		State.GravityVelocity += gravityAcceleration * DeltaTime;
		FVector displacement = (previousGravityVelocity + State.GravityVelocity) * (0.5 * DeltaTime);
		displacement += State.EngineVelocity * DeltaTime;
		State.Position += displacement;
		const float nextYawVelocity = FMath::FInterpConstantTo(State.YawAngularVelocity,
			0.0f, DeltaTime, engine.TurnDeceleration);
		const float nextPitchVelocity = FMath::FInterpConstantTo(State.PitchAngularVelocity,
			0.0f, DeltaTime, engine.TurnDeceleration);
		const float yawDelta = (State.YawAngularVelocity + nextYawVelocity) * 0.5f * DeltaTime;
		const float pitchDelta = (State.PitchAngularVelocity + nextPitchVelocity) * 0.5f * DeltaTime;
		State.Orientation = (State.Orientation * FRotator(pitchDelta, yawDelta, 0.0f).Quaternion()).GetNormalized();
		State.YawAngularVelocity = nextYawVelocity;
		State.PitchAngularVelocity = nextPitchVelocity;
		State.ElapsedSeconds += DeltaTime;
	}

	FNavigationControl ScalePredictedControl(const FNavigationControl& Control, double Fraction)
	{
		FNavigationControl scaled;
		scaled.ForwardFuel = static_cast<float>(Control.ForwardFuel * Fraction);
		scaled.RightFuel = static_cast<float>(Control.RightFuel * Fraction);
		scaled.UpFuel = static_cast<float>(Control.UpFuel * Fraction);
		scaled.YawFuel = static_cast<float>(Control.YawFuel * Fraction);
		scaled.PitchFuel = static_cast<float>(Control.PitchFuel * Fraction);
		return scaled;
	}

	bool TruncatePredictedArrival(const FNavigationState& Previous, const FPlannerContext& Context,
		const TArray<FVector>& CurrentBodyPositions, float DeltaTime,
		FNavigationState& State, FNavigationControl& Control)
	{
		if (FVector::DistSquared(Previous.Position, Context.Target) <= FMath::Square(ArrivalDistance))
		{
			State = Previous;
			Control = FNavigationControl();
			return true;
		}
		const FVector travel = State.Position - Previous.Position;
		const double fraction = travel.SizeSquared() > 0.0 ? FMath::Clamp(
			FVector::DotProduct(Context.Target - Previous.Position, travel) / travel.SizeSquared(), 0.0, 1.0) : 0.0;
		if (FVector::DistSquared(Previous.Position + travel * fraction, Context.Target) >
			FMath::Square(ArrivalDistance)) return false;
		const FNavigationControl fullControl = Control;
		const auto stateAtFraction = [&Previous, &Context, &CurrentBodyPositions, DeltaTime, &fullControl]
			(double arrivalFraction)
		{
			FNavigationState partialState = Previous;
			AdvanceSimulatedState(partialState, ScalePredictedControl(fullControl, arrivalFraction),
				Context, CurrentBodyPositions, static_cast<float>(DeltaTime * arrivalFraction));
			return partialState;
		};
		double lower = 0.0;
		double upper = 1.0;
		for (int32 refinement = 0; refinement < 32; ++refinement)
		{
			const double first = lower + (upper - lower) / 3.0;
			const double second = upper - (upper - lower) / 3.0;
			if (FVector::DistSquared(stateAtFraction(first).Position, Context.Target) <
				FVector::DistSquared(stateAtFraction(second).Position, Context.Target)) upper = second;
			else lower = first;
		}
		upper = (lower + upper) * 0.5;
		if (FVector::DistSquared(stateAtFraction(upper).Position, Context.Target) >
			FMath::Square(ArrivalDistance)) return false;
		lower = 0.0;
		for (int32 refinement = 0; refinement < 24; ++refinement)
		{
			const double middle = (lower + upper) * 0.5;
			if (FVector::DistSquared(stateAtFraction(middle).Position, Context.Target) <=
				FMath::Square(ArrivalDistance)) upper = middle;
			else lower = middle;
		}
		State = stateAtFraction(upper);
		Control = ScalePredictedControl(fullControl, upper);
		return true;
	}

	double EvaluateRouteBodyRisk(const FPlannerContext& Context, const FPlannerBody& Body,
		const FNavigationState& Current, double MinimumDistanceUU, double RequiredGapUU,
		double ClosingSpeedUUPerSecond, double ShipSpeedUUPerSecond, FRouteCandidate& Candidate)
	{
		const double escapeSpeed = UnrealUnitsPerMeter * FMath::Sqrt(2.0 *
			Context.GravityCoefficient * Body.MassTonnes /
			FMath::Max(1.0, Body.RadiusUU / UnrealUnitsPerMeter));
		const double closingSpeedRisk = ClosingSpeedUUPerSecond /
			FMath::Max(1.0, ClosingSpeedUUPerSecond + escapeSpeed);
		const double speedAdvantageRisk = CalculateShipToPlanetSpeedRisk(Context, Body, ShipSpeedUUPerSecond);
		const double speedFactor = closingSpeedRisk + (1.0 - closingSpeedRisk) * speedAdvantageRisk;
		const double fuelFactor = Context.InitialFuelKg > 0.0
			? 1.0 - Current.FuelKg / Context.InitialFuelKg : 1.0;
		const double proximity = Body.bPlanet
			? 1.0 / (1.0 + FMath::Square(MinimumDistanceUU / Body.AvoidRadiusUU))
			: FMath::Square(Body.AvoidRadiusUU /
				FMath::Max(Body.AvoidRadiusUU, MinimumDistanceUU));
		const double marginUU = FMath::Max(0.0, MinimumDistanceUU - Body.RadiusUU - RequiredGapUU);
		const double velocityErrorUUPerSecond = ShipSpeedUUPerSecond * Context.ManeuverVelocityErrorFraction;
		// Use the fixed correction window for the single maneuver error estimate.
		const double deviationUU = velocityErrorUUPerSecond * CorrectionWindowSeconds;
		const double sensitivity = deviationUU > 0.0
			? (marginUU > 0.0 ? FMath::Clamp(deviationUU / marginUU, 0.0, 1.0) : 1.0) : 0.0;
		// Two ideal correction impulses restore both position and velocity.
		const double correctionDeltaV = velocityErrorUUPerSecond +
			2.0 * deviationUU / CorrectionWindowSeconds;
		// Bound the sum of three axis fuel quotas; no correction fuel is actually spent here.
		const double correctionFuelKg = FMath::Sqrt(3.0) * correctionDeltaV /
			FMath::Max(UE_DOUBLE_SMALL_NUMBER, static_cast<double>(Context.Engine.ThrustAcceleration));
		const double requiredReserveKg = FMath::Max(correctionFuelKg, Candidate.DesiredCorrectionFuelKg);
		const double reserveRisk = requiredReserveKg > 0.0
			? (Candidate.AvailableCorrectionFuelKg > 0.0
				? FMath::Clamp(requiredReserveKg / Candidate.AvailableCorrectionFuelKg, 0.0, 1.0) : 1.0) : 0.0;
		Candidate.DistanceRisk = FMath::Max(Candidate.DistanceRisk, proximity);
		Candidate.ApproachSpeedRisk = FMath::Max(Candidate.ApproachSpeedRisk, proximity * speedFactor);
		Candidate.SpeedAdvantageRisk = FMath::Max(Candidate.SpeedAdvantageRisk, proximity * speedAdvantageRisk);
		Candidate.ManeuverSensitivityRisk = FMath::Max(Candidate.ManeuverSensitivityRisk, proximity * sensitivity);
		Candidate.CorrectionFuelRisk = FMath::Max(Candidate.CorrectionFuelRisk, proximity * reserveRisk);
		Candidate.MaximumManeuverDeviationUU = FMath::Max(Candidate.MaximumManeuverDeviationUU, deviationUU);
		Candidate.MinimumManeuverMarginUU = FMath::Min(Candidate.MinimumManeuverMarginUU, marginUU);
		Candidate.RequiredCorrectionFuelKg = FMath::Max(Candidate.RequiredCorrectionFuelKg, correctionFuelKg);
		const double hazard = proximity * (0.5 + 0.25 * speedFactor + 0.25 * fuelFactor);
		const double maneuverRisk = 1.0 - (1.0 - 0.5 * proximity * sensitivity) *
			(1.0 - 0.5 * proximity * reserveRisk);
		return FMath::Clamp(hazard + (1.0 - hazard) * maneuverRisk, 0.0, 1.0);
	}

	bool EvaluateSafetyAndRisk(const FNavigationState& Previous, const FNavigationState& Current,
		const FPlannerContext& Context, const TArray<FVector>& PreviousBodyPositions,
		const TArray<FVector>& CurrentBodyPositions, double& OutStepRisk, int32& OutBlockingBodyIndex,
		double& OutMinimumStarClearanceUU, double& OutMinimumPlanetClearanceUU,
		double& OutPhysicalGapUU, double& OutRequiredGapUU, FRouteCandidate& Candidate)
	{
		OutStepRisk = 0.0;
		OutBlockingBodyIndex = INDEX_NONE;
		OutPhysicalGapUU = 0.0;
		OutRequiredGapUU = 0.0;
		const double stepSeconds = Current.ElapsedSeconds - Previous.ElapsedSeconds;
		const double shipSpeed = FMath::Max((Previous.EngineVelocity + Previous.GravityVelocity).Size(),
			(Current.EngineVelocity + Current.GravityVelocity).Size());
		for (int32 bodyIndex = 0; bodyIndex < Context.Bodies.Num(); ++bodyIndex)
		{
			const FPlannerBody& body = Context.Bodies[bodyIndex];
			const FVector previousRelative = Previous.Position - PreviousBodyPositions[bodyIndex];
			const FVector currentRelative = Current.Position - CurrentBodyPositions[bodyIndex];
			const FVector relativeStep = currentRelative - previousRelative;
			const double fraction = relativeStep.SizeSquared() > 0.0
				? FMath::Clamp(-FVector::DotProduct(previousRelative, relativeStep) /
					relativeStep.SizeSquared(), 0.0, 1.0) : 0.0;
			const double closestDistance = (previousRelative + relativeStep * fraction).Size();
			double& minimumClearanceUU = body.bPlanet
				? OutMinimumPlanetClearanceUU : OutMinimumStarClearanceUU;
			minimumClearanceUU = FMath::Min(minimumClearanceUU,
				closestDistance - body.RadiusUU);
			const double relativeSpeed = stepSeconds > 0.0
				? relativeStep.Size() / stepSeconds : 0.0;
			const double requiredGap = CalculateBodyPredictionReserveUU(body, relativeSpeed, stepSeconds);
			const double requiredDistance = body.RadiusUU + requiredGap;
			if (closestDistance < requiredDistance)
			{
				OutBlockingBodyIndex = bodyIndex;
				OutPhysicalGapUU = closestDistance - body.RadiusUU;
				OutRequiredGapUU = requiredGap;
				return false;
			}
			const double closingSpeed = stepSeconds > 0.0 && previousRelative.Size() > 0.0
				? FMath::Max(0.0, -FVector::DotProduct(previousRelative.GetSafeNormal(),
					relativeStep / stepSeconds)) : 0.0;
			OutStepRisk = FMath::Max(OutStepRisk, EvaluateRouteBodyRisk(Context, body, Current,
				closestDistance, requiredGap, closingSpeed, shipSpeed, Candidate));
		}
		return true;
	}

	double GetScriptedFuelKg(const FNavigationControl& Fuel)
	{
		return static_cast<double>(Fuel.ForwardFuel) + FMath::Abs(static_cast<double>(Fuel.RightFuel)) +
			FMath::Abs(static_cast<double>(Fuel.UpFuel)) + FMath::Abs(static_cast<double>(Fuel.YawFuel)) +
			FMath::Abs(static_cast<double>(Fuel.PitchFuel));
	}

	double GetScriptedFuelToleranceKg(double InitialFuelKg)
	{
		return FMath::Max(static_cast<double>(UE_SMALL_NUMBER),
			2.0 * FLT_EPSILON * FMath::Max(0.0, InitialFuelKg));
	}

	void AddScriptedControl(FNavigationControl& Total, const FNavigationControl& Fuel, double Fraction)
	{
		Total.ForwardFuel += static_cast<float>(Fuel.ForwardFuel * Fraction);
		Total.RightFuel += static_cast<float>(Fuel.RightFuel * Fraction);
		Total.UpFuel += static_cast<float>(Fuel.UpFuel * Fraction);
		Total.YawFuel += static_cast<float>(Fuel.YawFuel * Fraction);
		Total.PitchFuel += static_cast<float>(Fuel.PitchFuel * Fraction);
	}

	FScriptedPose EvaluateScriptedMotion(const TArray<FScriptedMotionSegment>& Segments,
		double Seconds, int32* InOutSegmentIndex)
	{
		FScriptedPose pose;
		if (Segments.IsEmpty()) return pose;
		int32 segmentIndex = InOutSegmentIndex != nullptr
			? FMath::Clamp(*InOutSegmentIndex, 0, Segments.Num() - 1) : 0;
		while (segmentIndex < Segments.Num() - 1 &&
			Seconds > Segments[segmentIndex].StartSeconds + Segments[segmentIndex].DurationSeconds)
			++segmentIndex;
		while (segmentIndex > 0 && Seconds < Segments[segmentIndex].StartSeconds) --segmentIndex;
		if (InOutSegmentIndex != nullptr) *InOutSegmentIndex = segmentIndex;
		const FScriptedMotionSegment& segment = Segments[segmentIndex];
		if (segment.bCurved) return EvaluateTimeCurvePose(segment, Seconds);
		const double elapsed = FMath::Clamp(Seconds - segment.StartSeconds, 0.0, segment.DurationSeconds);
		const double timeFraction = segment.DurationSeconds > 0.0 ? elapsed / segment.DurationSeconds : 1.0;
		const double distance = FMath::Clamp(segment.InitialSpeedUU * elapsed +
			0.5 * segment.AccelerationUU * elapsed * elapsed, 0.0, segment.DistanceUU);
		const double distanceFraction = segment.DistanceUU > 0.0 ? distance / segment.DistanceUU : 0.0;
		pose.Position = FMath::Lerp(segment.StartPosition, segment.EndPosition, distanceFraction);
		if (elapsed >= segment.DurationSeconds) pose.Position = segment.EndPosition;
		pose.Orientation = FQuat::Slerp(segment.StartOrientation, segment.EndOrientation,
			timeFraction).GetNormalized();
		pose.Velocity = (segment.EndPosition - segment.StartPosition).GetSafeNormal() *
			FMath::Max(0.0, segment.InitialSpeedUU + segment.AccelerationUU * elapsed);
		pose.Phase = segment.Phase;
		pose.NextPointIndex = segment.NextPointIndex;
		return pose;
	}

	FNavigationControl AccumulateScriptedControl(const TArray<FScriptedMotionSegment>& Segments,
		double FromSeconds, double ToSeconds)
	{
		FNavigationControl fuel;
		if (ToSeconds <= FromSeconds) return fuel;
		for (const FScriptedMotionSegment& segment : Segments)
		{
			if (segment.StartSeconds >= ToSeconds) break;
			const double overlap = FMath::Min(ToSeconds, segment.StartSeconds + segment.DurationSeconds) -
				FMath::Max(FromSeconds, segment.StartSeconds);
			if (overlap <= 0.0 || segment.DurationSeconds <= 0.0) continue;
			AddScriptedControl(fuel, segment.Fuel, overlap / segment.DurationSeconds);
		}
		return fuel;
	}

	bool AppendScriptedSegment(TArray<FScriptedMotionSegment>& Segments,
		const FVector& Start, const FVector& End, const FQuat& StartOrientation,
		const FQuat& EndOrientation, double DurationSeconds, double InitialSpeedUU,
		double AccelerationUU, const FNavigationControl& Fuel, EFlightPhase Phase, int32 NextPointIndex)
	{
		if (DurationSeconds <= 0.0) return FVector::DistSquared(Start, End) <= UE_DOUBLE_SMALL_NUMBER;
		if (!FMath::IsFinite(DurationSeconds) || !FMath::IsFinite(InitialSpeedUU) ||
			!FMath::IsFinite(AccelerationUU) || Start.ContainsNaN() || End.ContainsNaN()) return false;
		if (Fuel.ForwardFuel < 0.0f || Fuel.ForwardFuel > MaxForwardFuelPerSecond * DurationSeconds + KINDA_SMALL_NUMBER ||
			FMath::Abs(Fuel.RightFuel) > MaxLateralFuelPerSecond * DurationSeconds + KINDA_SMALL_NUMBER ||
			FMath::Abs(Fuel.UpFuel) > MaxLateralFuelPerSecond * DurationSeconds + KINDA_SMALL_NUMBER ||
			FMath::Abs(Fuel.YawFuel) > MaxTurnFuelPerSecond * DurationSeconds + KINDA_SMALL_NUMBER ||
			FMath::Abs(Fuel.PitchFuel) > MaxTurnFuelPerSecond * DurationSeconds + KINDA_SMALL_NUMBER) return false;
		FScriptedMotionSegment& segment = Segments.AddDefaulted_GetRef();
		segment.StartSeconds = Segments.Num() > 1
			? Segments[Segments.Num() - 2].StartSeconds + Segments[Segments.Num() - 2].DurationSeconds : 0.0;
		segment.DurationSeconds = DurationSeconds;
		segment.StartPosition = Start;
		segment.EndPosition = End;
		segment.StartOrientation = StartOrientation;
		segment.EndOrientation = EndOrientation;
		segment.InitialSpeedUU = InitialSpeedUU;
		segment.AccelerationUU = AccelerationUU;
		segment.DistanceUU = FVector::Dist(Start, End);
		segment.Fuel = Fuel;
		segment.Phase = Phase;
		segment.NextPointIndex = NextPointIndex;
		return true;
	}

	bool CalculateScriptedTurn(const FQuat& Start, const FQuat& End,
		const FEngineSnapshot& Engine, double& OutSeconds, FNavigationControl& OutFuel)
	{
		OutSeconds = 0.0;
		OutFuel = FNavigationControl();
		const FRotator relative = (Start.Inverse() * End).Rotator();
		const double angle = FMath::Max(FMath::RadiansToDegrees(Start.AngularDistance(End)),
			FMath::Max(FMath::Abs(relative.Yaw), FMath::Abs(relative.Pitch)));
		if (angle <= KINDA_SMALL_NUMBER) return true;
		const double impulse = Engine.TurnImpulsePerFuelUnit;
		const double deceleration = FMath::Max(0.0, static_cast<double>(Engine.TurnDeceleration));
		const double acceleration = impulse * MaxTurnFuelPerSecond - deceleration;
		if (impulse <= 0.0 || Engine.MaxTurnSpeed <= 0.0f || acceleration <= 0.0) return false;
		const double peakSpeed = FMath::Min(static_cast<double>(Engine.MaxTurnSpeed),
			FMath::Sqrt(angle * acceleration));
		OutSeconds = angle / peakSpeed + peakSpeed / acceleration;
		const double fuel = (2.0 * peakSpeed + deceleration * OutSeconds) / impulse;
		OutFuel.YawFuel = static_cast<float>(fuel * relative.Yaw / angle);
		OutFuel.PitchFuel = static_cast<float>(fuel * relative.Pitch / angle);
		return FMath::IsFinite(OutSeconds) && FMath::IsFinite(GetScriptedFuelKg(OutFuel));
	}

	bool AppendScriptedTurn(TArray<FScriptedMotionSegment>& Segments, const FVector& Position,
		const FQuat& Start, const FQuat& End, const FEngineSnapshot& Engine,
		EFlightPhase Phase, int32 NextPointIndex)
	{
		double duration = 0.0;
		FNavigationControl fuel;
		if (!CalculateScriptedTurn(Start, End, Engine, duration, fuel)) return false;
		return AppendScriptedSegment(Segments, Position, Position, Start, End,
			duration, 0.0, 0.0, fuel, Phase, NextPointIndex);
	}

	FNavigationControl CalculateScriptedSideFuel(const FVector& IncomingDirection,
		const FVector& OutgoingDirection, double SpeedUU, const FEngineSnapshot& Engine)
	{
		FNavigationControl fuel;
		const double neededFuel = FVector::Dist(IncomingDirection, OutgoingDirection) *
			SpeedUU / Engine.ThrustAcceleration;
		const FVector local = IncomingDirection.Rotation().Quaternion().UnrotateVector(OutgoingDirection);
		const double sideLength = FMath::Sqrt(local.Y * local.Y + local.Z * local.Z);
		if (sideLength <= KINDA_SMALL_NUMBER) fuel.RightFuel = static_cast<float>(neededFuel);
		else
		{
			fuel.RightFuel = static_cast<float>(neededFuel * local.Y / sideLength);
			fuel.UpFuel = static_cast<float>(neededFuel * local.Z / sideLength);
		}
		return fuel;
	}

	bool BuildScriptedFuelLeg(const FVector& Start, const FVector& End,
		const FEngineSnapshot& Engine, double CruiseSpeedUU, bool bFinalLeg,
		int32 NextPointIndex, TArray<FScriptedMotionSegment>& Segments)
	{
		const FVector direction = (End - Start).GetSafeNormal();
		const FQuat orientation = direction.Rotation().Quaternion();
		const double length = FVector::Dist(Start, End);
		const double acceleration = Engine.ThrustAcceleration * MaxForwardFuelPerSecond;
		// Reserve at least 75 percent of each leg for fuel-free coasting.
		const double speed = FMath::Min(CruiseSpeedUU,
			0.5 * FMath::Sqrt((bFinalLeg ? 2.0 : 1.0) * acceleration * length));
		if (speed <= 0.0) return false;
		const double accelerationSeconds = speed / acceleration;
		const double accelerationDistance = speed * speed / (2.0 * acceleration);
		const FVector accelerationEnd = accelerationDistance >= length
			? End : Start + direction * accelerationDistance;
		FNavigationControl accelerationFuel;
		accelerationFuel.ForwardFuel = static_cast<float>(speed / Engine.ThrustAcceleration);
		if (!AppendScriptedSegment(Segments, Start, accelerationEnd, orientation, orientation,
			accelerationSeconds, 0.0, acceleration, accelerationFuel, EFlightPhase::Launch, NextPointIndex)) return false;
		const double brakingDistance = bFinalLeg ? 0.0 : accelerationDistance;
		const FVector brakingStart = bFinalLeg ? End : End - direction * brakingDistance;
		const double coastDistance = FMath::Max(0.0, length - accelerationDistance - brakingDistance);
		if (coastDistance > UE_DOUBLE_SMALL_NUMBER &&
			!AppendScriptedSegment(Segments, accelerationEnd, brakingStart, orientation, orientation,
				coastDistance / speed, speed, 0.0, FNavigationControl(), EFlightPhase::Coast, NextPointIndex)) return false;
		if (!bFinalLeg && !AppendScriptedSegment(Segments, brakingStart, End, orientation, orientation,
			accelerationSeconds, speed, -acceleration, accelerationFuel, EFlightPhase::Brake, NextPointIndex)) return false;
		return true;
	}

	bool CalculateScriptedTimeSpeed(const TArray<FVector>& Points, const FEngineSnapshot& Engine,
		double RequestedSpeedUU, double& OutSpeedUU)
	{
		OutSpeedUU = FMath::Min(RequestedSpeedUU,
			FMath::Sqrt(2.0 * Engine.ThrustAcceleration * MaxForwardFuelPerSecond *
				FVector::Dist(Points[0], Points[1])));
		for (int32 pointIndex = 2; pointIndex < Points.Num(); ++pointIndex)
		{
			const FVector incoming = (Points[pointIndex - 1] - Points[pointIndex - 2]).GetSafeNormal();
			const FVector outgoing = (Points[pointIndex] - Points[pointIndex - 1]).GetSafeNormal();
			const double length = FVector::Dist(Points[pointIndex - 1], Points[pointIndex]);
			double turnSeconds = 0.0;
			FNavigationControl turnFuel;
			if (!CalculateScriptedTurn(incoming.Rotation().Quaternion(),
				outgoing.Rotation().Quaternion(), Engine, turnSeconds, turnFuel)) return false;
			if (turnSeconds > 0.0) OutSpeedUU = FMath::Min(OutSpeedUU, length / turnSeconds);
			const double sideFactor = FVector::Dist(incoming, outgoing) / Engine.ThrustAcceleration;
			if (sideFactor > 0.0) OutSpeedUU = FMath::Min(OutSpeedUU,
				FMath::Sqrt(length * MaxLateralFuelPerSecond / sideFactor));
		}
		return FMath::IsFinite(OutSpeedUU) && OutSpeedUU > 0.0;
	}

	bool BuildScriptedTimeLeg(const TArray<FVector>& Points, int32 PointIndex,
		const FEngineSnapshot& Engine, double SpeedUU, TArray<FScriptedMotionSegment>& Segments)
	{
		const FVector start = Points[PointIndex - 1];
		const FVector end = Points[PointIndex];
		const FVector direction = (end - start).GetSafeNormal();
		const FQuat orientation = direction.Rotation().Quaternion();
		const double length = FVector::Dist(start, end);
		FVector cruiseStart = start;
		if (PointIndex == 1)
		{
			const double acceleration = Engine.ThrustAcceleration * MaxForwardFuelPerSecond;
			const double duration = SpeedUU / acceleration;
			const double distance = SpeedUU * duration * 0.5;
			cruiseStart = distance >= length ? end : start + direction * distance;
			FNavigationControl fuel;
			fuel.ForwardFuel = static_cast<float>(MaxForwardFuelPerSecond * duration);
			if (!AppendScriptedSegment(Segments, start, cruiseStart, orientation, orientation,
				duration, 0.0, acceleration, fuel, EFlightPhase::Launch, PointIndex)) return false;
		}
		else
		{
			const FVector incoming = (start - Points[PointIndex - 2]).GetSafeNormal();
			const FQuat previousOrientation = incoming.Rotation().Quaternion();
			double duration = 0.0;
			FNavigationControl fuel;
			if (!CalculateScriptedTurn(previousOrientation, orientation, Engine, duration, fuel)) return false;
			const FNavigationControl sideFuel = CalculateScriptedSideFuel(incoming, direction, SpeedUU, Engine);
			AddScriptedControl(fuel, sideFuel, 1.0);
			duration = FMath::Max(duration, FMath::Max(FMath::Abs(static_cast<double>(fuel.RightFuel)),
				FMath::Abs(static_cast<double>(fuel.UpFuel))) / MaxLateralFuelPerSecond);
			const double distance = SpeedUU * duration;
			if (distance > length + KINDA_SMALL_NUMBER) return false;
			cruiseStart = distance >= length ? end : start + direction * distance;
			fuel.ForwardFuel = static_cast<float>(MaxForwardFuelPerSecond * duration);
			if (!AppendScriptedSegment(Segments, start, cruiseStart, previousOrientation, orientation,
				duration, SpeedUU, 0.0, fuel, EFlightPhase::Redirect, PointIndex)) return false;
		}
		const double cruiseDistance = FVector::Dist(cruiseStart, end);
		if (cruiseDistance <= UE_DOUBLE_SMALL_NUMBER) return true;
		const double duration = cruiseDistance / SpeedUU;
		FNavigationControl fuel;
		fuel.ForwardFuel = static_cast<float>(MaxForwardFuelPerSecond * duration);
		return AppendScriptedSegment(Segments, cruiseStart, end, orientation, orientation,
			duration, SpeedUU, 0.0, fuel, EFlightPhase::Launch, PointIndex);
	}

	bool BuildScriptedSchedule(const FPlannerContext& Context, EPlanFamily Family,
		double CruiseSpeedFraction, FRouteCandidate& Candidate)
	{
		TArray<FVector> points;
		points.Add(Context.InitialState.Position);
		for (int32 pointIndex = 1; pointIndex < Candidate.Guidance.Num(); ++pointIndex)
		{
			const FVector point = pointIndex == Candidate.Guidance.Num() - 1
				? Context.Target : Candidate.Guidance[pointIndex];
			if (FVector::DistSquared(points.Last(), point) > UE_DOUBLE_SMALL_NUMBER) points.Add(point);
		}
		if (points.Num() < 2 || Context.Engine.ThrustAcceleration <= 0.0f ||
			Context.Engine.MaxSpeed <= 0.0f) return false;
		Candidate.Guidance = points;
		FQuat previousOrientation = Context.InitialState.Orientation;
		const FVector initialVelocity = Context.InitialState.EngineVelocity + Context.InitialState.GravityVelocity;
		if (!initialVelocity.IsNearlyZero())
		{
			const FQuat brakeOrientation = (-initialVelocity).Rotation().Quaternion();
			if (!AppendScriptedTurn(Candidate.MotionSegments, points[0], previousOrientation,
				brakeOrientation, Context.Engine, EFlightPhase::Align, 1)) return false;
			FNavigationControl fuel;
			fuel.ForwardFuel = static_cast<float>(initialVelocity.Size() / Context.Engine.ThrustAcceleration);
			if (!AppendScriptedSegment(Candidate.MotionSegments, points[0], points[0],
				brakeOrientation, brakeOrientation, fuel.ForwardFuel / MaxForwardFuelPerSecond,
				0.0, 0.0, fuel, EFlightPhase::Brake, 1)) return false;
			previousOrientation = brakeOrientation;
		}
		double cruiseSpeed = Context.Engine.MaxSpeed * CruiseSpeedFraction;
		if (Family == EPlanFamily::ContinuousCorrection &&
			!CalculateScriptedTimeSpeed(points, Context.Engine, cruiseSpeed, cruiseSpeed)) return false;
		for (int32 pointIndex = 1; pointIndex < points.Num(); ++pointIndex)
		{
			const FQuat orientation = (points[pointIndex] - points[pointIndex - 1]).Rotation().Quaternion();
			if (Family == EPlanFamily::SparseBurns || pointIndex == 1)
			{
				if (!AppendScriptedTurn(Candidate.MotionSegments, points[pointIndex - 1],
					previousOrientation, orientation, Context.Engine,
					pointIndex == 1 ? EFlightPhase::Align : EFlightPhase::Redirect, pointIndex)) return false;
			}
			const bool bLegBuilt = Family == EPlanFamily::SparseBurns
				? BuildScriptedFuelLeg(points[pointIndex - 1], points[pointIndex], Context.Engine,
					cruiseSpeed, pointIndex == points.Num() - 1, pointIndex, Candidate.MotionSegments)
				: BuildScriptedTimeLeg(points, pointIndex, Context.Engine, cruiseSpeed, Candidate.MotionSegments);
			if (!bLegBuilt) return false;
			previousOrientation = orientation;
		}
		if (Candidate.MotionSegments.IsEmpty()) return false;
		FScriptedMotionSegment& finalSegment = Candidate.MotionSegments.Last();
		finalSegment.EndPosition = Context.Target;
		finalSegment.DistanceUU = FVector::Dist(finalSegment.StartPosition, Context.Target);
		for (const FScriptedMotionSegment& segment : Candidate.MotionSegments)
		{
			Candidate.FuelUsedKg += GetScriptedFuelKg(segment.Fuel);
			Candidate.ForwardFuelUsedKg += segment.Fuel.ForwardFuel;
			Candidate.LateralFuelUsedKg += FMath::Abs(segment.Fuel.RightFuel) + FMath::Abs(segment.Fuel.UpFuel);
			Candidate.TurnFuelUsedKg += FMath::Abs(segment.Fuel.YawFuel) + FMath::Abs(segment.Fuel.PitchFuel);
			if (GetScriptedFuelKg(segment.Fuel) > 0.0)
			{
				Candidate.BurnDurationSeconds += segment.DurationSeconds;
				FPredictedStep& maneuver = Candidate.Maneuvers.AddDefaulted_GetRef();
				maneuver.TimeSeconds = segment.StartSeconds;
				maneuver.Position = segment.StartPosition;
				maneuver.Control = segment.Fuel;
			}
			else Candidate.CoastDurationSeconds += segment.DurationSeconds;
		}
		const FScriptedMotionSegment& last = Candidate.MotionSegments.Last();
		Candidate.FlightTimeSeconds = last.StartSeconds + last.DurationSeconds;
		return FMath::IsFinite(Candidate.FuelUsedKg) && FMath::IsFinite(Candidate.FlightTimeSeconds);
	}

	bool SimulateCandidate(const TArray<FVector>& Guidance, const FPlannerContext& Context,
		EPlanFamily Family, double CruiseSpeedFraction, FRouteCandidate& OutCandidate,
		FBlockedPath* OutBlocked, FSimulationFailure* OutFailure)
	{
		FNavigationState state = Context.InitialState;
		const auto reject = [&state, &Context, OutFailure](ESimulationFailureReason Reason, int32 BodyIndex)
		{
			if (OutFailure != nullptr)
			{
				OutFailure->Reason = Reason;
				OutFailure->ElapsedSeconds = state.ElapsedSeconds;
				OutFailure->TargetDistanceUU = FVector::Dist(state.Position, Context.Target);
				OutFailure->RemainingFuelKg = state.FuelKg;
				OutFailure->BodyIndex = BodyIndex;
			}
			return false;
		};
		OutCandidate = FRouteCandidate();
		OutCandidate.Family = Family;
		OutCandidate.CruiseSpeedFraction = CruiseSpeedFraction;
		if (Guidance.Num() < 2 || !FMath::IsFinite(CruiseSpeedFraction) ||
			CruiseSpeedFraction <= 0.0 || CruiseSpeedFraction > 1.0)
			return reject(ESimulationFailureReason::InvalidInput, INDEX_NONE);
		OutCandidate.Guidance = Guidance;
		OutCandidate.MacroTurnCount = CountGuideTurns(Guidance);
		if (Family == EPlanFamily::SparseBurns && OutCandidate.MacroTurnCount > Context.FuelMaxTurns)
			return reject(ESimulationFailureReason::InvalidInput, INDEX_NONE);
		if (!BuildScriptedSchedule(Context, Family, CruiseSpeedFraction, OutCandidate))
			return reject(ESimulationFailureReason::InvalidInput, INDEX_NONE);
		if (OutCandidate.FuelUsedKg > Context.InitialFuelKg)
		{
			if (OutFailure != nullptr) OutFailure->RequiredFuelKg = OutCandidate.FuelUsedKg;
			return reject(ESimulationFailureReason::Fuel, INDEX_NONE);
		}
		// Only fuel left after the complete nominal schedule is available for corrections.
		OutCandidate.AvailableCorrectionFuelKg = FMath::Max(0.0, Context.InitialFuelKg - OutCandidate.FuelUsedKg);
		OutCandidate.DesiredCorrectionFuelKg = OutCandidate.FuelUsedKg * Context.DesiredCorrectionFuelReserveFraction;
		int32 segmentIndex = 0;
		const FScriptedPose initial = EvaluateScriptedMotion(OutCandidate.MotionSegments, 0.0, &segmentIndex);
		state.Position = initial.Position;
		state.Orientation = initial.Orientation;
		state.EngineVelocity = initial.Velocity;
		state.GravityVelocity = FVector::ZeroVector;
		state.YawAngularVelocity = 0.0f;
		state.PitchAngularVelocity = 0.0f;
		state.ElapsedSeconds = 0.0;
		OutCandidate.PredictedPath.Add(state.Position);
		FPredictedStateSample& initialSample = OutCandidate.PredictedStates.AddDefaulted_GetRef();
		initialSample.Position = state.Position;
		initialSample.Velocity = state.EngineVelocity;
		double peakRisk = 0.0;
		double accumulatedRisk = 0.0;
		double spentFuelKg = 0.0;
		TArray<FVector> currentBodyPositions;
		TArray<FVector> nextBodyPositions;
		CachePredictedBodyPositions(Context, 0.0, currentBodyPositions);
		for (int32 stepIndex = 0; stepIndex < MaximumPredictionSteps; ++stepIndex)
		{
			OutCandidate.PredictionSteps = stepIndex + 1;
			const FScriptedMotionSegment& segment = OutCandidate.MotionSegments[segmentIndex];
			const double segmentEnd = segment.StartSeconds + segment.DurationSeconds;
			const double nextSeconds = FMath::Min(segmentEnd, FMath::Min(OutCandidate.FlightTimeSeconds,
				state.ElapsedSeconds + GetPredictionStepSeconds(state, Context, currentBodyPositions)));
			if (nextSeconds <= state.ElapsedSeconds) return reject(ESimulationFailureReason::InvalidStep, INDEX_NONE);
			const FNavigationState previous = state;
			FNavigationControl fuel;
			AddScriptedControl(fuel, segment.Fuel,
				(nextSeconds - state.ElapsedSeconds) / segment.DurationSeconds);
			spentFuelKg += GetScriptedFuelKg(fuel);
			const FScriptedPose pose = EvaluateScriptedMotion(OutCandidate.MotionSegments, nextSeconds, &segmentIndex);
			state.Position = pose.Position;
			state.Orientation = pose.Orientation;
			state.EngineVelocity = pose.Velocity;
			state.NextPointIndex = pose.NextPointIndex;
			state.ElapsedSeconds = nextSeconds;
			state.FuelKg = FMath::Max(0.0, Context.InitialFuelKg - spentFuelKg);
			if (state.Position.ContainsNaN() || state.EngineVelocity.ContainsNaN())
				return reject(ESimulationFailureReason::NonFiniteState, INDEX_NONE);
			CachePredictedBodyPositions(Context, nextSeconds, nextBodyPositions);
			double stepRisk = 0.0;
			int32 blockingBodyIndex = INDEX_NONE;
			double physicalGapUU = 0.0;
			double requiredGapUU = 0.0;
			if (!EvaluateSafetyAndRisk(previous, state, Context, currentBodyPositions, nextBodyPositions,
				stepRisk, blockingBodyIndex, OutCandidate.MinimumStarClearanceUU,
				OutCandidate.MinimumPlanetClearanceUU, physicalGapUU, requiredGapUU, OutCandidate))
			{
				if (OutFailure != nullptr)
				{
					OutFailure->PhysicalGapUU = physicalGapUU;
					OutFailure->RequiredGapUU = requiredGapUU;
				}
				if (!Context.Bodies.IsValidIndex(blockingBodyIndex))
					return reject(ESimulationFailureReason::InvalidStep, blockingBodyIndex);
				const bool bPlanetCollision = Context.Bodies[blockingBodyIndex].bPlanet;
				if (OutBlocked != nullptr && bPlanetCollision)
				{
					OutBlocked->Guidance = OutCandidate.Guidance;
					OutBlocked->BodyIndex = blockingBodyIndex;
					OutBlocked->NextPointIndex = state.NextPointIndex;
					OutBlocked->TimeSeconds = nextSeconds;
				}
				return reject(bPlanetCollision ? ESimulationFailureReason::PlanetCollision
					: ESimulationFailureReason::StarCollision, blockingBodyIndex);
			}
			peakRisk = FMath::Max(peakRisk, stepRisk);
			accumulatedRisk += stepRisk * (nextSeconds - previous.ElapsedSeconds);
			const bool bArrived = nextSeconds >= OutCandidate.FlightTimeSeconds;
			const bool bSegmentEnd = nextSeconds >= segmentEnd;
			if ((bArrived || bSegmentEnd || FVector::DistSquared(OutCandidate.PredictedPath.Last(),
				state.Position) >= FMath::Square(10000.0)) &&
				FVector::DistSquared(OutCandidate.PredictedPath.Last(), state.Position) > UE_DOUBLE_SMALL_NUMBER)
				OutCandidate.PredictedPath.Add(state.Position);
			if (bArrived || bSegmentEnd || nextSeconds >=
				OutCandidate.PredictedStates.Last().TimeSeconds + ReferenceSampleIntervalSeconds)
			{
				FPredictedStateSample& sample = OutCandidate.PredictedStates.AddDefaulted_GetRef();
				sample.TimeSeconds = nextSeconds;
				sample.Position = state.Position;
				sample.Velocity = state.EngineVelocity;
			}
			Swap(currentBodyPositions, nextBodyPositions);
			if (bSegmentEnd && !bArrived) ++segmentIndex;
			if (!bArrived) continue;
			OutCandidate.RiskCost = 0.5 * peakRisk + 0.5 * accumulatedRisk /
				FMath::Max(0.001, OutCandidate.FlightTimeSeconds);
			return true;
		}
		return reject(ESimulationFailureReason::Unverified, INDEX_NONE);
	}

	struct FPredictionRequest
	{
		int32 GuideIndex = INDEX_NONE;
		EPlanFamily Family = EPlanFamily::ContinuousCorrection;
		double CruiseFraction = 1.0;
		double EstimatedFuelKg = 0.0;
		double EstimatedTimeSeconds = 0.0;
		double Cost = 0.0;
		bool bTried = false;
	};

	struct FSearchDiagnostics
	{
		FSimulationFailureSummary Failures[2];
		int32 Attempts[2] = {};
		int32 Successes[2] = {};
		int32 Duplicates[2] = {};
		int64 Steps = 0;
		double FineSeconds = 0.0;
	};

	void BuildPredictionRequests(const FPlannerContext& Context,
		const TArray<FHierarchicalGuide>& Guides, TArray<FPredictionRequest>& OutRequests);
	void SimulateRankedRequests(const FPlannerContext& Context,
		const TArray<FHierarchicalGuide>& Guides, TArray<FPredictionRequest>& Requests,
		FRouteSearchResults& OutResults, FSearchDiagnostics& OutDiagnostics);
	void LogSearchDiagnostics(const FPlannerContext& Context, const FRouteSearchResults& Results,
		const FSearchDiagnostics& Diagnostics, int32 RequestCount, bool bLog);

	void SearchRoutes(const FPlannerContext& Context, FRouteSearchResults& OutResults, bool bLog)
	{
		OutResults.InitialFuelKg = Context.InitialFuelKg;
		TArray<FHierarchicalGuide> guides;
		BuildHierarchicalGuides(Context, guides, bLog);
		OutResults.AttemptedPaths = guides.Num();
		TArray<FPredictionRequest> requests;
		BuildPredictionRequests(Context, guides, requests);
		FSearchDiagnostics diagnostics;
		SimulateRankedRequests(Context, guides, requests, OutResults, diagnostics);
		LogSearchDiagnostics(Context, OutResults, diagnostics, requests.Num(), bLog);
	}

	double GetPathLength(const TArray<FVector>& Points)
	{
		double length = 0.0;
		for (int32 pointIndex = 1; pointIndex < Points.Num(); ++pointIndex)
			length += FVector::Dist(Points[pointIndex - 1], Points[pointIndex]);
		return length;
	}

	FVector SamplePathDistance(const TArray<FVector>& Points, double DistanceUU)
	{
		for (int32 pointIndex = 1; pointIndex < Points.Num(); ++pointIndex)
		{
			const FVector segment = Points[pointIndex] - Points[pointIndex - 1];
			const double length = segment.Size();
			if (length <= 0.0) continue;
			if (DistanceUU <= length)
				return Points[pointIndex - 1] + segment * (DistanceUU / length);
			DistanceUU -= length;
		}
		return Points.Last();
	}

	double DistanceToPathSquared(const FVector& Location, const TArray<FVector>& Points)
	{
		double distanceSquared = TNumericLimits<double>::Max();
		for (int32 pointIndex = 1; pointIndex < Points.Num(); ++pointIndex)
		{
			const FVector start = Points[pointIndex - 1];
			const FVector segment = Points[pointIndex] - start;
			const double fraction = segment.SizeSquared() > 0.0
				? FMath::Clamp(FVector::DotProduct(Location - start, segment) /
					segment.SizeSquared(), 0.0, 1.0) : 0.0;
			distanceSquared = FMath::Min(distanceSquared,
				FVector::DistSquared(Location, start + segment * fraction));
		}
		return distanceSquared;
	}

	bool ArePredictedPathsDistinct(const TArray<FVector>& First, const TArray<FVector>& Second,
		double SeparationUU)
	{
		if (First.Num() < 2 || Second.Num() < 2) return false;
		const double firstLength = GetPathLength(First);
		const double secondLength = GetPathLength(Second);
		double squaredSeparation = 0.0;
		constexpr int32 ShapeSamples = 24;
		for (int32 sampleIndex = 1; sampleIndex < ShapeSamples; ++sampleIndex)
		{
			const double fraction = static_cast<double>(sampleIndex) / ShapeSamples;
			squaredSeparation += DistanceToPathSquared(
				SamplePathDistance(First, firstLength * fraction), Second);
			squaredSeparation += DistanceToPathSquared(
				SamplePathDistance(Second, secondLength * fraction), First);
		}
		const double meanSquaredSeparation = squaredSeparation / (2.0 * (ShapeSamples - 1));
		return meanSquaredSeparation > FMath::Square(FMath::Max(ArrivalDistance, SeparationUU));
	}

	bool AreGuidesEqual(const TArray<FVector>& First, const TArray<FVector>& Second)
	{
		if (First.Num() != Second.Num()) return false;
		for (int32 pointIndex = 0; pointIndex < First.Num(); ++pointIndex)
			if (FVector::DistSquared(First[pointIndex], Second[pointIndex]) > 1.0) return false;
		return true;
	}

	void BuildPredictionRequests(const FPlannerContext& Context,
		const TArray<FHierarchicalGuide>& Guides, TArray<FPredictionRequest>& OutRequests)
	{
		double minimumFuel = TNumericLimits<double>::Max();
		double maximumFuel = 0.0;
		double minimumTime = TNumericLimits<double>::Max();
		double maximumTime = 0.0;
		for (int32 guideIndex = 0; guideIndex < Guides.Num(); ++guideIndex)
		{
			const FHierarchicalGuide& guide = Guides[guideIndex];
			const EPlanFamily families[] = {EPlanFamily::SparseBurns, EPlanFamily::ContinuousCorrection};
			for (const EPlanFamily family : families)
			{
				if (guide.Style == EGuideStyle::Fuel && family != EPlanFamily::SparseBurns) continue;
				if (guide.Style == EGuideStyle::Time && family != EPlanFamily::ContinuousCorrection) continue;
				if (family == EPlanFamily::SparseBurns && guide.BendCount > Context.FuelMaxTurns) continue;
				for (const double cruiseFraction : Context.CruiseSpeedFractions)
				{
					const double coarseFraction = GetCoarseCruiseSpeed(Context, guide.Style) /
						FMath::Max(1.0, static_cast<double>(Context.Engine.MaxSpeed));
					const double speedRatio = cruiseFraction / FMath::Max(0.01, coarseFraction);
					FPredictionRequest& request = OutRequests.AddDefaulted_GetRef();
					request.GuideIndex = guideIndex;
					request.Family = family;
					request.CruiseFraction = cruiseFraction;
					request.EstimatedFuelKg = guide.EstimatedFuelKg * speedRatio;
					request.EstimatedTimeSeconds = guide.EstimatedTimeSeconds / speedRatio;
					if (family == EPlanFamily::ContinuousCorrection)
						request.EstimatedFuelKg += MaxForwardFuelPerSecond * request.EstimatedTimeSeconds;
					minimumFuel = FMath::Min(minimumFuel, request.EstimatedFuelKg);
					maximumFuel = FMath::Max(maximumFuel, request.EstimatedFuelKg);
					minimumTime = FMath::Min(minimumTime, request.EstimatedTimeSeconds);
					maximumTime = FMath::Max(maximumTime, request.EstimatedTimeSeconds);
				}
			}
		}
		for (FPredictionRequest& request : OutRequests)
		{
			const FHierarchicalGuide& guide = Guides[request.GuideIndex];
			const FSpaceNavRouteWeightSet& weights = Context.ModeWeights[static_cast<int32>(guide.Style)];
			const double fuelCost = maximumFuel > minimumFuel
				? (request.EstimatedFuelKg - minimumFuel) / (maximumFuel - minimumFuel) : 0.0;
			const double timeCost = maximumTime > minimumTime
				? (request.EstimatedTimeSeconds - minimumTime) / (maximumTime - minimumTime) : 0.0;
			request.Cost = weights.FuelWeight * fuelCost + weights.TimeWeight * timeCost +
				weights.RiskWeight * guide.EstimatedRiskCost;
		}
	}

	int32 FindNextPredictionRequest(const TArray<FHierarchicalGuide>& Guides,
		const TArray<FPredictionRequest>& Requests, int32 StyleIndex, bool bPreferNewGuide)
	{
		int32 bestIndex = INDEX_NONE;
		double bestCost = TNumericLimits<double>::Max();
		for (int32 requestIndex = 0; requestIndex < Requests.Num(); ++requestIndex)
		{
			const FPredictionRequest& request = Requests[requestIndex];
			if (request.bTried || static_cast<int32>(Guides[request.GuideIndex].Style) != StyleIndex) continue;
			bool bAlreadySimulated = false;
			bool bGuideUsed = false;
			for (const FPredictionRequest& earlier : Requests)
			{
				if (!earlier.bTried || !AreGuidesEqual(Guides[earlier.GuideIndex].Points,
					Guides[request.GuideIndex].Points)) continue;
				bGuideUsed = true;
				if (earlier.Family == request.Family &&
					FMath::IsNearlyEqual(earlier.CruiseFraction, request.CruiseFraction, 0.0001))
					bAlreadySimulated = true;
			}
			if (bAlreadySimulated || (bPreferNewGuide && bGuideUsed)) continue;
			if (request.Cost >= bestCost) continue;
			bestCost = request.Cost;
			bestIndex = requestIndex;
		}
		return bestIndex;
	}

	void SimulateRankedRequests(const FPlannerContext& Context,
		const TArray<FHierarchicalGuide>& Guides, TArray<FPredictionRequest>& Requests,
		FRouteSearchResults& OutResults, FSearchDiagnostics& OutDiagnostics)
	{
		const double startedSeconds = FPlatformTime::Seconds();
		for (int32 attemptIndex = 0; attemptIndex < Context.MaxDetailedCandidates; ++attemptIndex)
		{
			int32 requestIndex = INDEX_NONE;
			const bool bPreferNewGuide = attemptIndex < Context.MaxDetailedCandidates / 2;
			for (int32 offset = 0; offset < 3 && requestIndex == INDEX_NONE; ++offset)
				requestIndex = FindNextPredictionRequest(Guides, Requests,
					(attemptIndex + offset) % 3, bPreferNewGuide);
			if (requestIndex == INDEX_NONE && bPreferNewGuide)
				for (int32 offset = 0; offset < 3 && requestIndex == INDEX_NONE; ++offset)
					requestIndex = FindNextPredictionRequest(Guides, Requests,
						(attemptIndex + offset) % 3, false);
			if (requestIndex == INDEX_NONE) break;
			FPredictionRequest& request = Requests[requestIndex];
			request.bTried = true;
			++OutResults.DetailedAttempts;
			const int32 familyIndex = static_cast<int32>(request.Family);
			++OutDiagnostics.Attempts[familyIndex];
			FRouteCandidate candidate;
			FSimulationFailure failure;
			const bool bSafe = SimulateCandidate(Guides[request.GuideIndex].Points, Context,
				request.Family, request.CruiseFraction, candidate, nullptr, &failure);
			OutDiagnostics.Steps += candidate.PredictionSteps;
			if (bSafe)
			{
				++OutDiagnostics.Successes[familyIndex];
				const int32 previousCount = OutResults.Candidates.Num();
				AddDistinctCandidate(OutResults.Candidates, MoveTemp(candidate));
				if (previousCount == OutResults.Candidates.Num())
				{
					++OutResults.DuplicatePlans;
					++OutDiagnostics.Duplicates[familyIndex];
				}
			}
			else
			{
				OutDiagnostics.Failures[familyIndex].Add(failure);
				if (failure.Reason == ESimulationFailureReason::Unverified) ++OutResults.UnverifiedPaths;
			}
		}
		OutDiagnostics.FineSeconds = FPlatformTime::Seconds() - startedSeconds;
	}

	void LogSearchDiagnostics(const FPlannerContext& Context, const FRouteSearchResults& Results,
		const FSearchDiagnostics& Diagnostics, int32 RequestCount, bool bLog)
	{
		if (!bLog) return;
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Profiles %d"), RequestCount);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Fine %d %.1fms"),
			Results.DetailedAttempts, Diagnostics.FineSeconds * 1000.0);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Fine budget %d"), Context.MaxDetailedCandidates);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Sparse tried %d"), Diagnostics.Attempts[0]);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Sparse safe %d"), Diagnostics.Successes[0]);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Sparse dup %d"), Diagnostics.Duplicates[0]);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Corrected tried %d"), Diagnostics.Attempts[1]);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Corrected safe %d"), Diagnostics.Successes[1]);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Corrected dup %d"), Diagnostics.Duplicates[1]);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Fine steps %lld"), static_cast<long long>(Diagnostics.Steps));
		Diagnostics.Failures[0].Log(TEXT("Sparse"), true, Context);
		Diagnostics.Failures[1].Log(TEXT("Corrected"), true, Context);
	}


	void SnapshotPredictionSettings(const USpaceNavRouteOptimizationDataAsset& Settings,
		FPlannerContext& OutContext)
	{
		OutContext.FuelMaxTurns = Settings.FuelMaxTurns;
		OutContext.CruiseSpeedFractions = Settings.CruiseSpeedFractions;
		OutContext.MaxGuideCandidates = Settings.MaxGuideCandidates;
		OutContext.MaxDetailedCandidates = Settings.MaxDetailedCandidates;
		OutContext.MinimumRouteSeparationFraction = Settings.MinimumRouteSeparationFraction;
		OutContext.CriticalBodyClearanceRadiusFraction = FMath::IsFinite(Settings.CriticalBodyClearanceRadiusFraction)
			? FMath::Max(0.0, Settings.CriticalBodyClearanceRadiusFraction) : 0.6;
		OutContext.ShipToPlanetSpeedFactor = FMath::IsFinite(Settings.ShipToPlanetSpeedFactor)
			? Settings.ShipToPlanetSpeedFactor : 2.0;
		OutContext.DesiredCorrectionFuelReserveFraction = FMath::IsFinite(Settings.DesiredCorrectionFuelReservePercent)
			? FMath::Clamp(Settings.DesiredCorrectionFuelReservePercent, 0.0, 100.0) / 100.0 : 0.2;
		OutContext.ManeuverVelocityErrorFraction = FMath::IsFinite(Settings.ManeuverSpeedErrorFraction)
			? FMath::Clamp(Settings.ManeuverSpeedErrorFraction, 0.0, 1.0) : 0.01;
		OutContext.ModeWeights[0] = Settings.FuelEfficient;
		OutContext.ModeWeights[1] = Settings.TimeEfficient;
		OutContext.ModeWeights[2] = Settings.Balanced;
		for (FSpaceNavRouteWeightSet& weights : OutContext.ModeWeights)
		{
			const double sum = weights.FuelWeight + weights.TimeWeight + weights.RiskWeight;
			weights.FuelWeight /= sum;
			weights.TimeWeight /= sum;
			weights.RiskWeight /= sum;
		}
	}

	bool AddDistinctPath(TArray<TArray<FVector>>& Paths, const TArray<FVector>& Path)
	{
		for (const TArray<FVector>& existing : Paths)
		{
			if (existing.Num() != Path.Num()) continue;
			bool bSamePath = true;
			for (int32 pointIndex = 0; pointIndex < Path.Num(); ++pointIndex)
			{
				if (FVector::DistSquared(existing[pointIndex], Path[pointIndex]) <= 1.0) continue;
				bSamePath = false;
				break;
			}
			if (bSamePath) return false;
		}
		Paths.Add(Path);
		return true;
	}

	void AddDistinctCandidate(TArray<FRouteCandidate>& Candidates, FRouteCandidate&& Candidate)
	{
		for (const FRouteCandidate& existing : Candidates)
		{
			if (!FMath::IsNearlyEqual(existing.FuelUsedKg, Candidate.FuelUsedKg, 0.001) ||
				!FMath::IsNearlyEqual(existing.FlightTimeSeconds, Candidate.FlightTimeSeconds, 0.001) ||
				!FMath::IsNearlyEqual(existing.RiskCost, Candidate.RiskCost, 0.0001)) continue;
			if (!ArePredictedPathsDistinct(existing.PredictedPath, Candidate.PredictedPath,
				ArrivalDistance)) return;
		}
		Candidates.Add(MoveTemp(Candidate));
	}

	void SelectDistinctModeCandidates(const TArray<FRouteCandidate>& Candidates,
		const FSpaceNavRouteWeightSet* WeightSets, int32 FuelMaxTurns,
		double SeparationUU, int32* OutIndices, double* OutCosts)
	{
		double minimumFuel = TNumericLimits<double>::Max();
		double maximumFuel = 0.0;
		double minimumTime = TNumericLimits<double>::Max();
		double maximumTime = 0.0;
		for (const FRouteCandidate& candidate : Candidates)
		{
			minimumFuel = FMath::Min(minimumFuel, candidate.FuelUsedKg);
			maximumFuel = FMath::Max(maximumFuel, candidate.FuelUsedKg);
			minimumTime = FMath::Min(minimumTime, candidate.FlightTimeSeconds);
			maximumTime = FMath::Max(maximumTime, candidate.FlightTimeSeconds);
		}
		for (int32 modeIndex = 0; modeIndex < 3; ++modeIndex)
		{
			OutIndices[modeIndex] = INDEX_NONE;
			OutCosts[modeIndex] = TNumericLimits<double>::Max();
			FSpaceNavRouteWeightSet weights = WeightSets[modeIndex];
			const double sum = weights.FuelWeight + weights.TimeWeight + weights.RiskWeight;
			if (!FMath::IsFinite(sum) || sum <= 0.0) continue;
			weights.FuelWeight /= sum;
			weights.TimeWeight /= sum;
			weights.RiskWeight /= sum;
			double bestCost = TNumericLimits<double>::Max();
			for (int32 candidateIndex = 0; candidateIndex < Candidates.Num(); ++candidateIndex)
			{
				const FRouteCandidate& candidate = Candidates[candidateIndex];
				if (modeIndex == 0 && (candidate.Family != EPlanFamily::SparseBurns ||
					candidate.MacroTurnCount > FuelMaxTurns ||
					candidate.CoastDurationSeconds < ReferenceSampleIntervalSeconds)) continue;
				if (modeIndex == 1 && candidate.Family != EPlanFamily::ContinuousCorrection) continue;
				bool bDistinct = true;
				for (int32 earlierMode = 0; earlierMode < modeIndex; ++earlierMode)
				{
					if (OutIndices[earlierMode] == INDEX_NONE) continue;
					if (ArePredictedPathsDistinct(candidate.PredictedPath,
						Candidates[OutIndices[earlierMode]].PredictedPath, SeparationUU)) continue;
					bDistinct = false;
					break;
				}
				if (!bDistinct) continue;
				const double cost = GetCandidateCost(candidate, weights,
					minimumFuel, maximumFuel, minimumTime, maximumTime);
				if (cost >= bestCost) continue;
				bestCost = cost;
				OutIndices[modeIndex] = candidateIndex;
				OutCosts[modeIndex] = cost;
			}
		}
	}

	double GetCandidateCost(const FRouteCandidate& Candidate, const FSpaceNavRouteWeightSet& Weights,
		double MinimumFuelKg, double MaximumFuelKg, double MinimumTimeSeconds, double MaximumTimeSeconds)
	{
		const double fuelRange = MaximumFuelKg - MinimumFuelKg;
		const double fuelCost = fuelRange > 0.0
			? (Candidate.FuelUsedKg - MinimumFuelKg) / fuelRange : 0.0;
		const double timeRange = MaximumTimeSeconds - MinimumTimeSeconds;
		const double timeCost = timeRange > 0.0
			? (Candidate.FlightTimeSeconds - MinimumTimeSeconds) / timeRange : 0.0;
		return Weights.FuelWeight * fuelCost + Weights.TimeWeight * timeCost +
			Weights.RiskWeight * Candidate.RiskCost;
	}

	void LogSelectedPlanDetails(const FRouteCandidate& Candidate, bool bLog)
	{
		if (!bLog) return;
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Cruise %.2f"), Candidate.CruiseSpeedFraction);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Fuel kg %.3f"), Candidate.FuelUsedKg);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Flight s %.2f"), Candidate.FlightTimeSeconds);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Risk %.4f"), Candidate.RiskCost);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Risk dist %.3f"), Candidate.DistanceRisk);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Risk speed %.3f"), Candidate.ApproachSpeedRisk);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Risk ratio %.3f"), Candidate.SpeedAdvantageRisk);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Risk error %.3f"), Candidate.ManeuverSensitivityRisk);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Risk fuel %.3f"), Candidate.CorrectionFuelRisk);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Error max m %.3g"),
			Candidate.MaximumManeuverDeviationUU / UnrealUnitsPerMeter);
		if (Candidate.MinimumManeuverMarginUU != TNumericLimits<double>::Max())
			UE_LOG(LogSpaceNavRoute, Display, TEXT("Margin min m %.3g"),
				Candidate.MinimumManeuverMarginUU / UnrealUnitsPerMeter);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Corr need kg %.3g"), Candidate.RequiredCorrectionFuelKg);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Corr free kg %.3g"), Candidate.AvailableCorrectionFuelKg);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Corr goal kg %.3g"), Candidate.DesiredCorrectionFuelKg);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Turns %d"), Candidate.MacroTurnCount);
		int32 arcs = 0;
		for (const FScriptedMotionSegment& segment : Candidate.MotionSegments)
			if (segment.bCurved) arcs = FMath::Max(arcs, segment.CurveFuelGroup + 1);
		if (arcs > 0) UE_LOG(LogSpaceNavRoute, Display, TEXT("Arcs %d"), arcs);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Burn s %.2f"), Candidate.BurnDurationSeconds);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Coast s %.2f"), Candidate.CoastDurationSeconds);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Main kg %.3f"), Candidate.ForwardFuelUsedKg);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Side kg %.3f"), Candidate.LateralFuelUsedKg);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Turn kg %.3f"), Candidate.TurnFuelUsedKg);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Steps %d"), Candidate.PredictionSteps);
		if (Candidate.MinimumStarClearanceUU != TNumericLimits<double>::Max())
			UE_LOG(LogSpaceNavRoute, Display, TEXT("Star gap m %.3g"),
				Candidate.MinimumStarClearanceUU / UnrealUnitsPerMeter);
		if (Candidate.MinimumPlanetClearanceUU != TNumericLimits<double>::Max())
			UE_LOG(LogSpaceNavRoute, Display, TEXT("Planet gap m %.3g"),
				Candidate.MinimumPlanetClearanceUU / UnrealUnitsPerMeter);
	}
}

namespace
{
	const TCHAR* GetAutopilotPhaseName(EFlightPhase Phase)
	{
		switch (Phase)
		{
		case EFlightPhase::Align: return TEXT("Aim");
		case EFlightPhase::Launch: return TEXT("Burn");
		case EFlightPhase::Coast: return TEXT("Coast");
		case EFlightPhase::Brake: return TEXT("Brake");
		case EFlightPhase::Redirect: return TEXT("Turn");
		default: return TEXT("Other");
		}
	}

	const TCHAR* GetAutopilotReasonName(uint8 ReasonMask)
	{
		// The mask classifies the combined command, not separate fuel grants.
		const TCHAR* names[] = {
			TEXT("Script"), TEXT("Track"), TEXT("Avoid"), TEXT("Mix")
		};
		return names[ReasonMask & 3];
	}

	void ResetAutopilotTelemetry(FAutopilotTelemetry& Telemetry, double InitialFuelKg)
	{
		Telemetry = FAutopilotTelemetry();
		Telemetry.InitialFuelKg = InitialFuelKg;
		Telemetry.LastFuelKg = InitialFuelKg;
	}

	void ObserveAutopilotTelemetry(FAutopilotTelemetry& Telemetry,
		EFlightPhase Phase, uint8 ReasonMask, float DeltaTime)
	{
		const uint8 reason = ReasonMask & 3;
		if (!Telemetry.bHasState)
		{
			Telemetry.WindowFirstPhase = Phase;
			Telemetry.WindowFirstReason = reason;
			Telemetry.bHasState = true;
		}
		else
		{
			if (Phase != Telemetry.LastPhase) ++Telemetry.PhaseChanges;
			if (reason != Telemetry.LastReason) ++Telemetry.ReasonChanges;
		}
		Telemetry.LastPhase = Phase;
		Telemetry.LastReason = reason;
		const double duration = FMath::Max(0.0, static_cast<double>(DeltaTime));
		Telemetry.WindowSeconds += duration;
		Telemetry.ReasonSeconds[reason] += duration;
	}

	void RecordAutopilotBroadcast(FSpaceNavFuelRequest& RequestEvent,
		USpaceNavResourceStoreComponent& Resources, FAutopilotTelemetry& Telemetry,
		int32 ActuatorIndex, double RequestedKg, double CappedKg, float DeltaTime)
	{
		FActuatorTelemetry& axis = Telemetry.Axes[ActuatorIndex];
		axis.CommandedKg += RequestedKg;
		axis.CappedKg += CappedKg;
		Telemetry.FlightCommandedKg += RequestedKg;
		Telemetry.FlightCappedKg += CappedKg;
		if (CappedKg <= 0.0) return;

		const double beforeFuelKg = Resources.Resources.Fuel;
		RequestEvent.Broadcast(static_cast<float>(CappedKg));
		const double afterFuelKg = Resources.Resources.Fuel;
		const double observedSpentKg = FMath::Max(0.0, beforeFuelKg - afterFuelKg);
		const float issuedFuelKg = static_cast<float>(CappedKg);
		const float expectedFuelKg = static_cast<float>(beforeFuelKg) - issuedFuelKg;
		if (issuedFuelKg <= beforeFuelKg && afterFuelKg == expectedFuelKg)
			Telemetry.FuelRoundingKg += observedSpentKg - issuedFuelKg;
		axis.ObservedSpentKg += observedSpentKg;
		axis.ActiveSeconds += FMath::Max(0.0, static_cast<double>(DeltaTime));
		++axis.Count;
		Telemetry.FlightObservedSpentKg += observedSpentKg;
		Telemetry.ReasonSpentKg[Telemetry.LastReason] += observedSpentKg;
		Telemetry.LastFuelKg = afterFuelKg;
	}

	bool BroadcastAutopilotControl(ASpaceNavPawn& Pawn,
		USpaceNavResourceStoreComponent& Resources, FAutopilotTelemetry& Telemetry,
		const FNavigationControl& Requested, const FNavigationControl& Capped, float DeltaTime,
		TFunctionRef<bool()> IsCurrentFlight)
	{
		// Preserve the existing delegate order.
		FSpaceNavFuelRequest* events[AutopilotActuatorCount] = {
			&Pawn.OnTurnYawRight, &Pawn.OnTurnYawLeft,
			&Pawn.OnTurnPitchUp, &Pawn.OnTurnPitchDown,
			&Pawn.OnMoveRight, &Pawn.OnMoveLeft,
			&Pawn.OnMoveUp, &Pawn.OnMoveDown, &Pawn.OnMoveForward
		};
		const float requestedFuel[AutopilotActuatorCount] = {
			FMath::Max(0.0f, Requested.YawFuel),
			FMath::Max(0.0f, -Requested.YawFuel),
			FMath::Max(0.0f, Requested.PitchFuel),
			FMath::Max(0.0f, -Requested.PitchFuel),
			FMath::Max(0.0f, Requested.RightFuel),
			FMath::Max(0.0f, -Requested.RightFuel),
			FMath::Max(0.0f, Requested.UpFuel),
			FMath::Max(0.0f, -Requested.UpFuel),
			FMath::Max(0.0f, Requested.ForwardFuel)
		};
		const float cappedFuel[AutopilotActuatorCount] = {
			FMath::Max(0.0f, Capped.YawFuel),
			FMath::Max(0.0f, -Capped.YawFuel),
			FMath::Max(0.0f, Capped.PitchFuel),
			FMath::Max(0.0f, -Capped.PitchFuel),
			FMath::Max(0.0f, Capped.RightFuel),
			FMath::Max(0.0f, -Capped.RightFuel),
			FMath::Max(0.0f, Capped.UpFuel),
			FMath::Max(0.0f, -Capped.UpFuel),
			FMath::Max(0.0f, Capped.ForwardFuel)
		};
		for (int32 axisIndex = 0; axisIndex < AutopilotActuatorCount; ++axisIndex)
		{
			if (!IsCurrentFlight()) return false;
			RecordAutopilotBroadcast(*events[axisIndex], Resources, Telemetry,
				axisIndex, requestedFuel[axisIndex], cappedFuel[axisIndex], DeltaTime);
			if (!IsCurrentFlight()) return false;
		}
		return true;
	}

	void FlushAutopilotTelemetry(FAutopilotTelemetry& Telemetry,
		const FAutopilotSnapshot& Snapshot, bool bLog, bool bForce)
	{
		Telemetry.LastFuelKg = Snapshot.FuelKg;
		if (!bForce && Telemetry.WindowSeconds < 1.0) return;
		if (Telemetry.WindowSeconds <= 0.0) return;

		const TCHAR* axisNames[AutopilotActuatorCount] = {
			TEXT("YR"), TEXT("YL"), TEXT("PU"), TEXT("PD"),
			TEXT("R"), TEXT("L"), TEXT("U"), TEXT("D"), TEXT("Main")
		};
		if (bLog)
		{
			UE_LOG(LogSpaceNavRoute, Display, TEXT("Time %.3gs"), Snapshot.ElapsedSeconds);
			UE_LOG(LogSpaceNavRoute, Display, TEXT("Seg %d/%d"),
				Snapshot.SegmentIndex, Snapshot.SegmentCount);
			UE_LOG(LogSpaceNavRoute, Display, TEXT("Speed %.3g m/s"),
				Snapshot.SpeedUUPerSecond / UnrealUnitsPerMeter);
			UE_LOG(LogSpaceNavRoute, Display, TEXT("Fuel %.3gkg"), Snapshot.FuelKg);
			UE_LOG(LogSpaceNavRoute, Display, TEXT("Goal %.3gkm"),
				Snapshot.GoalDistanceUU / (UnrealUnitsPerMeter * 1000.0));
			UE_LOG(LogSpaceNavRoute, Display, TEXT("Cross %.3gm"),
				Snapshot.CrossLineErrorUU / UnrealUnitsPerMeter);
			UE_LOG(LogSpaceNavRoute, Display, TEXT("Slip %.3gdeg"),
				Snapshot.HeadingVelocityAngleDegrees);
			UE_LOG(LogSpaceNavRoute, Display, TEXT("Yaw %.3g deg/s"),
				Snapshot.YawRateDegreesPerSecond);
			UE_LOG(LogSpaceNavRoute, Display, TEXT("Pitch %.3g deg/s"),
				Snapshot.PitchRateDegreesPerSecond);
			UE_LOG(LogSpaceNavRoute, Display, TEXT("Ph %s>%s %u"),
				GetAutopilotPhaseName(Telemetry.WindowFirstPhase),
				GetAutopilotPhaseName(Telemetry.LastPhase), Telemetry.PhaseChanges);
			UE_LOG(LogSpaceNavRoute, Display, TEXT("Why %s>%s"),
				GetAutopilotReasonName(Telemetry.WindowFirstReason),
				GetAutopilotReasonName(Telemetry.LastReason));
			UE_LOG(LogSpaceNavRoute, Display, TEXT("Why changes %u"),
				Telemetry.ReasonChanges);
		}
		for (int32 axisIndex = 0; axisIndex < AutopilotActuatorCount; ++axisIndex)
		{
			FActuatorTelemetry& axis = Telemetry.Axes[axisIndex];
			if (bLog && (axis.CommandedKg > 0.0 || axis.CappedKg > 0.0))
			{
				// Q: requested kg, C: capped kg, S: observed spent kg.
				// T: issued-command duration, N: delegate broadcasts.
				UE_LOG(LogSpaceNavRoute, Display, TEXT("%s Q%.2g C%.2g"),
					axisNames[axisIndex], axis.CommandedKg, axis.CappedKg);
				UE_LOG(LogSpaceNavRoute, Display, TEXT("%s S%.2g T%.2g"),
					axisNames[axisIndex], axis.ObservedSpentKg, axis.ActiveSeconds);
				UE_LOG(LogSpaceNavRoute, Display, TEXT("%s N%u"),
					axisNames[axisIndex], axis.Count);
			}
			axis = FActuatorTelemetry();
		}
		for (int32 reasonIndex = 0; reasonIndex < 4; ++reasonIndex)
		{
			if (bLog && Telemetry.ReasonSeconds[reasonIndex] > 0.0)
			{
				UE_LOG(LogSpaceNavRoute, Display, TEXT("R %s %.3gkg"),
					GetAutopilotReasonName(static_cast<uint8>(reasonIndex)),
					Telemetry.ReasonSpentKg[reasonIndex]);
			}
			Telemetry.ReasonSeconds[reasonIndex] = 0.0;
			Telemetry.ReasonSpentKg[reasonIndex] = 0.0;
		}
		Telemetry.WindowSeconds = 0.0;
		Telemetry.PhaseChanges = 0;
		Telemetry.ReasonChanges = 0;
		Telemetry.WindowFirstPhase = Telemetry.LastPhase;
		Telemetry.WindowFirstReason = Telemetry.LastReason;
	}

	void LogAutopilotSummary(const FAutopilotTelemetry& Telemetry,
		const TCHAR* EndReason, bool bLog)
	{
		// Use short fixed reasons: arrived, reset, impact, fuel, invalid.
		const bool bFailure = FCString::Strcmp(EndReason, TEXT("fuel")) == 0 ||
			FCString::Strcmp(EndReason, TEXT("invalid")) == 0;
		if (bFailure)
		{
			UE_LOG(LogSpaceNavRoute, Warning, TEXT("Auto end %s"), EndReason);
		}
		else if (bLog)
		{
			UE_LOG(LogSpaceNavRoute, Display, TEXT("Auto end %s"), EndReason);
		}
		if (!bLog) return;
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Sum Q%.2g C%.2g"),
			Telemetry.FlightCommandedKg, Telemetry.FlightCappedKg);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Total spent %.3gkg"),
			Telemetry.FlightObservedSpentKg);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Round %.3gkg"), Telemetry.FuelRoundingKg);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Bank %.3g/%.3g"),
			Telemetry.InitialFuelKg, Telemetry.LastFuelKg);
	}
}



namespace
{
	FNavigationState SnapshotNavigationState(const ASpaceNavPawn& Pawn,
		const USpaceNavEngineComponent& Engine,
		const USpaceNavResourceStoreComponent& Resources, double ElapsedSeconds)
	{
		FNavigationState state;
		state.Position = Pawn.GetActorLocation();
		state.Orientation = Pawn.GetActorQuat();
		state.EngineVelocity = Engine.GetLinearVelocity();
		state.GravityVelocity = Engine.GetGravityVelocity();
		state.YawAngularVelocity = Engine.GetYawAngularVelocity();
		state.PitchAngularVelocity = Engine.GetPitchAngularVelocity();
		state.FuelKg = Resources.Resources.Fuel;
		state.ElapsedSeconds = ElapsedSeconds;
		return state;
	}

	bool HasReachedFlightTarget(const FVector& Position, const FVector& PreviousPosition,
		bool bHasPreviousPosition, const FVector& Target)
	{
		const FVector travel = Position - PreviousPosition;
		const double fraction = bHasPreviousPosition && !travel.IsNearlyZero()
			? FMath::Clamp(FVector::DotProduct(Target - PreviousPosition, travel) /
				travel.SizeSquared(), 0.0, 1.0) : 1.0;
		const FVector closest = bHasPreviousPosition
			? PreviousPosition + travel * fraction : Position;
		return FVector::DistSquared(closest, Target) <= FMath::Square(ArrivalDistance);
	}

	FVector ProjectSpatialSegment(const FVector& Start, const FVector& End,
		const FVector& Location, double& OutFraction)
	{
		const FVector segment = End - Start;
		OutFraction = segment.SizeSquared() > 0.0
			? FMath::Clamp(FVector::DotProduct(Location - Start, segment) /
				segment.SizeSquared(), 0.0, 1.0) : 0.0;
		return Start + segment * OutFraction;
	}

	template<typename PointType, typename LocationAccessor>
	FVector ProjectForwardSpatialPath(const TArray<PointType>& Points,
		const FVector& Location, double SearchDistanceUU,
		int32& NextPointIndex, double& OutFraction, LocationAccessor GetLocation)
	{
		NextPointIndex = FMath::Clamp(NextPointIndex, 1, Points.Num() - 1);
		FVector closest = ProjectSpatialSegment(GetLocation(Points[NextPointIndex - 1]),
			GetLocation(Points[NextPointIndex]), Location, OutFraction);
		double closestSquared = FVector::DistSquared(closest, Location);
		double searchedDistanceUU = 0.0;
		while (NextPointIndex < Points.Num() - 1 && searchedDistanceUU <= SearchDistanceUU)
		{
			double nextFraction = 0.0;
			const FVector nextProjection = ProjectSpatialSegment(
				GetLocation(Points[NextPointIndex]), GetLocation(Points[NextPointIndex + 1]),
				Location, nextFraction);
			const double nextSquared = FVector::DistSquared(nextProjection, Location);
			if (nextSquared >= closestSquared) break;
			searchedDistanceUU += FVector::Dist(GetLocation(Points[NextPointIndex]),
				GetLocation(Points[NextPointIndex + 1]));
			++NextPointIndex;
			closest = nextProjection;
			closestSquared = nextSquared;
			OutFraction = nextFraction;
		}
		return closest;
	}

	FSpatialRouteReference FindSpatialRouteReference(const FRouteCandidate& Candidate,
		const FVector& Position, double SearchDistanceUU,
		int32& NextPathPointIndex, int32& NextStatePointIndex)
	{
		FSpatialRouteReference reference;
		reference.Position = Position;
		if (Candidate.PredictedPath.Num() < 2 || Candidate.PredictedStates.Num() < 2)
			return reference;
		double pathFraction = 0.0;
		reference.Position = ProjectForwardSpatialPath(Candidate.PredictedPath,
			Position, SearchDistanceUU, NextPathPointIndex, pathFraction,
			[](const FVector& Point) { return Point; });
		reference.Direction = (Candidate.PredictedPath[NextPathPointIndex] -
			Candidate.PredictedPath[NextPathPointIndex - 1]).GetSafeNormal();
		reference.CrossLineErrorUU = FVector::Dist(reference.Position, Position);
		// Correlate speed with the drawn line, independently of the flight clock.
		double stateFraction = 0.0;
		ProjectForwardSpatialPath(Candidate.PredictedStates, reference.Position,
			SearchDistanceUU, NextStatePointIndex, stateFraction,
			[](const FPredictedStateSample& Sample) { return Sample.Position; });
		const FPredictedStateSample& previous = Candidate.PredictedStates[NextStatePointIndex - 1];
		const FPredictedStateSample& next = Candidate.PredictedStates[NextStatePointIndex];
		reference.Velocity = FMath::Lerp(previous.Velocity, next.Velocity, stateFraction);
		reference.TimeSeconds = FMath::Lerp(previous.TimeSeconds, next.TimeSeconds, stateFraction);
		reference.NextStatePointIndex = NextStatePointIndex;
		return reference;
	}

	FFlightFeedback BuildTrackingFeedback(const FRouteCandidate& Candidate,
		const FSpatialRouteReference& Reference, const FNavigationState& State,
		double TargetDistanceUU)
	{
		FFlightFeedback feedback;
		const FVector positionError = Reference.Position - State.Position;
		const FVector velocityError = Reference.Velocity -
			(State.EngineVelocity + State.GravityVelocity);
		const FVector crossVelocityError = velocityError - Reference.Direction *
			FVector::DotProduct(velocityError, Reference.Direction);
		const double clearance = FMath::Min(Candidate.MinimumStarClearanceUU,
			Candidate.MinimumPlanetClearanceUU);
		const double tolerance = FMath::Max(ArrivalDistance, FMath::Min(clearance * 0.5,
			TargetDistanceUU * 0.02 + ArrivalDistance));
		feedback.bTracking = (positionError + crossVelocityError * CorrectionWindowSeconds).Size() >
			tolerance;
		feedback.TargetVelocity = Reference.Velocity + positionError / CorrectionWindowSeconds;
		return feedback;
	}

	template<typename MonitoredBodyType>
	FLiveFlightThreat FindLiveFlightThreat(const TArray<MonitoredBodyType>& Bodies,
		const FNavigationState& State, const FVector& GravityAcceleration,
		const FSpatialRouteReference& Reference, const FRouteCandidate& Candidate,
		const FEngineSnapshot& Engine, float DeltaTime)
	{
		FLiveFlightThreat threat;
		const double horizon = LiveThreatLookAheadSeconds;
		const FVector velocity = State.EngineVelocity + State.GravityVelocity;
		const FVector projectedEnd = State.Position + velocity * horizon +
			GravityAcceleration * (0.5 * horizon * horizon);
		const FVector positionError = Reference.Position - State.Position;
		const FVector velocityError = Reference.Velocity - velocity;
		double worstIntrusionRatio = 0.0;
		for (const MonitoredBodyType& body : Bodies)
		{
			AActor* actor = body.Actor.Get();
			if (actor == nullptr) continue;
			const FVector center = actor->GetActorLocation();
			const ASpaceNavPlanet* planet = body.bStar ? nullptr : Cast<ASpaceNavPlanet>(actor);
			const FVector futureCenter = planet != nullptr
				? planet->PredictLocationAfter(horizon) : center;
			const FVector relativeStart = State.Position - center;
			const FVector relativeStep = projectedEnd - futureCenter - relativeStart;
			const double fraction = relativeStep.SizeSquared() > 0.0
				? FMath::Clamp(-FVector::DotProduct(relativeStart, relativeStep) /
					relativeStep.SizeSquared(), 0.0, 1.0) : 0.0;
			const double closestDistance = (relativeStart + relativeStep * fraction).Size();
			const double requiredDistance = body.RadiusUU + CalculateTurnReserveUU(
				body.RadiusUU, relativeStep.Size() / horizon, Engine);
			if (closestDistance >= requiredDistance ||
				(fraction <= 0.0 && FVector::DotProduct(relativeStart, relativeStep) >= 0.0 &&
					relativeStart.Size() > body.RadiusUU)) continue;
			const double intrusion = requiredDistance - closestDistance;
			const double ratio = intrusion / requiredDistance;
			if (ratio <= worstIntrusionRatio) continue;
			const double closestTime = horizon * fraction;
			const FVector bodyAtClosest = planet != nullptr
				? planet->PredictLocationAfter(closestTime) : center;
			int32 futureIndex = Reference.NextStatePointIndex;
			const FVector referenceAtClosest = InterpolatePredictedState(Candidate.PredictedStates,
				Reference.TimeSeconds + closestTime, futureIndex).Position;
			const FVector expectedAtClosest = referenceAtClosest - positionError -
				velocityError * closestTime;
			const double hardDistance = body.RadiusUU + CalculatePredictionReserveUU(
				body.RadiusUU, relativeStep.Size() / horizon, DeltaTime);
			if (FVector::Dist(expectedAtClosest, bodyAtClosest) >= hardDistance) continue;
			const FVector shipAtClosest = State.Position + velocity * closestTime +
				GravityAcceleration * (0.5 * closestTime * closestTime);
			const FVector travelDirection = velocity.GetSafeNormal();
			FVector outward = shipAtClosest - bodyAtClosest;
			if (outward.IsNearlyZero()) outward = referenceAtClosest - bodyAtClosest;
			FVector lateral = outward - travelDirection *
				FVector::DotProduct(outward, travelDirection);
			if (lateral.IsNearlyZero())
			{
				const FVector preferred = referenceAtClosest - bodyAtClosest;
				lateral = preferred - travelDirection *
					FVector::DotProduct(preferred, travelDirection);
			}
			if (lateral.IsNearlyZero())
				lateral = State.Orientation.RotateVector(FVector::RightVector);
			worstIntrusionRatio = ratio;
			threat.bDetected = true;
			threat.bStar = body.bStar;
			threat.Direction = lateral.GetSafeNormal();
			threat.GapUU = closestDistance - body.RadiusUU;
			threat.MinimumVelocityChangeUU = 2.0 * intrusion / FMath::Max(0.25, closestTime);
			const double usableRate = FMath::Min(MaxLateralFuelPerSecond,
				State.FuelKg / FMath::Max(0.25, closestTime));
			const double reachable = 0.5 * Engine.ThrustAcceleration * usableRate *
				FMath::Square(FMath::Max(0.25, closestTime));
			threat.bUnreachable = intrusion > reachable;
		}
		return threat;
	}

	void LimitFlightControlRates(FNavigationControl& Control, float DeltaTime)
	{
		const float turnLimit = static_cast<float>(MaxTurnFuelPerSecond * DeltaTime);
		const float lateralLimit = static_cast<float>(MaxLateralFuelPerSecond * DeltaTime);
		Control.YawFuel = FMath::Clamp(Control.YawFuel, -turnLimit, turnLimit);
		Control.PitchFuel = FMath::Clamp(Control.PitchFuel, -turnLimit, turnLimit);
		Control.RightFuel = FMath::Clamp(Control.RightFuel, -lateralLimit, lateralLimit);
		Control.UpFuel = FMath::Clamp(Control.UpFuel, -lateralLimit, lateralLimit);
		Control.ForwardFuel = FMath::Clamp(Control.ForwardFuel, 0.0f,
			static_cast<float>(MaxForwardFuelPerSecond * DeltaTime));
	}

	FAutopilotSnapshot MakeAutopilotSnapshot(const FNavigationState& State,
		double FuelKg, const FVector& Target, double CrossLineErrorUU)
	{
		FAutopilotSnapshot snapshot;
		snapshot.ElapsedSeconds = State.ElapsedSeconds;
		const FVector velocity = State.EngineVelocity + State.GravityVelocity;
		snapshot.SpeedUUPerSecond = velocity.Size();
		snapshot.FuelKg = FuelKg;
		snapshot.GoalDistanceUU = FVector::Dist(State.Position, Target);
		snapshot.CrossLineErrorUU = CrossLineErrorUU;
		if (!velocity.IsNearlyZero())
		{
			const FVector forward = State.Orientation.RotateVector(FVector::ForwardVector);
			snapshot.HeadingVelocityAngleDegrees = FMath::RadiansToDegrees(FMath::Acos(
				FMath::Clamp(FVector::DotProduct(forward, velocity.GetSafeNormal()), -1.0, 1.0)));
		}
		snapshot.YawRateDegreesPerSecond = State.YawAngularVelocity;
		snapshot.PitchRateDegreesPerSecond = State.PitchAngularVelocity;
		return snapshot;
	}
}

namespace
{
	FVector EvaluateTimeCurvePoint(const FTimeCurveGeometry& Curve, double Parameter)
	{
		const double inverse = 1.0 - Parameter;
		return Curve.StartPosition * (inverse * inverse * inverse) +
			Curve.Control1 * (3.0 * inverse * inverse * Parameter) +
			Curve.Control2 * (3.0 * inverse * Parameter * Parameter) +
			Curve.EndPosition * (Parameter * Parameter * Parameter);
	}

	FVector EvaluateTimeCurveTangent(const FTimeCurveGeometry& Curve, double Parameter)
	{
		const double inverse = 1.0 - Parameter;
		return (Curve.Control1 - Curve.StartPosition) * (3.0 * inverse * inverse) +
			(Curve.Control2 - Curve.Control1) * (6.0 * inverse * Parameter) +
			(Curve.EndPosition - Curve.Control2) * (3.0 * Parameter * Parameter);
	}

	double FindTimeCurveParameter(const FTimeCurveGeometry& Curve, double DistanceUU)
	{
		if (Curve.ArcLengths.Num() < 2) return 0.0;
		const double distance = FMath::Clamp(DistanceUU, 0.0, Curve.ArcLengths.Last());
		int32 lower = 0;
		int32 upper = Curve.ArcLengths.Num() - 1;
		while (upper - lower > 1)
		{
			const int32 middle = (lower + upper) / 2;
			if (Curve.ArcLengths[middle] < distance) lower = middle;
			else upper = middle;
		}
		const double span = Curve.ArcLengths[upper] - Curve.ArcLengths[lower];
		const double fraction = span > 0.0 ? (distance - Curve.ArcLengths[lower]) / span : 0.0;
		return (lower + fraction) / (Curve.ArcLengths.Num() - 1);
	}

	FScriptedPose EvaluateTimeCurvePose(const FScriptedMotionSegment& Segment, double Seconds)
	{
		FScriptedPose pose;
		if (!Segment.Curve.IsValid()) return pose;
		const double elapsed = FMath::Clamp(Seconds - Segment.StartSeconds, 0.0, Segment.DurationSeconds);
		const double distance = FMath::Clamp(Segment.InitialSpeedUU * elapsed +
			0.5 * Segment.AccelerationUU * elapsed * elapsed, 0.0, Segment.DistanceUU);
		const double parameter = FindTimeCurveParameter(*Segment.Curve, Segment.CurveStartDistanceUU + distance);
		const FVector tangent = EvaluateTimeCurveTangent(*Segment.Curve, parameter).GetSafeNormal();
		pose.Position = elapsed >= Segment.DurationSeconds
			? Segment.EndPosition : EvaluateTimeCurvePoint(*Segment.Curve, parameter);
		pose.Orientation = tangent.Rotation().Quaternion();
		pose.Velocity = tangent * FMath::Max(0.0, Segment.InitialSpeedUU + Segment.AccelerationUU * elapsed);
		pose.Phase = Segment.Phase;
		pose.NextPointIndex = Segment.NextPointIndex;
		return pose;
	}

	void SplitTimeCurveControls(const FVector* Points, double Fraction, FVector* Left, FVector* Right)
	{
		const FVector first = FMath::Lerp(Points[0], Points[1], Fraction);
		const FVector second = FMath::Lerp(Points[1], Points[2], Fraction);
		const FVector third = FMath::Lerp(Points[2], Points[3], Fraction);
		const FVector firstSecond = FMath::Lerp(first, second, Fraction);
		const FVector secondThird = FMath::Lerp(second, third, Fraction);
		const FVector middle = FMath::Lerp(firstSecond, secondThird, Fraction);
		Left[0] = Points[0]; Left[1] = first; Left[2] = firstSecond; Left[3] = middle;
		Right[0] = middle; Right[1] = secondThird; Right[2] = third; Right[3] = Points[3];
	}

	double GetTimeCurveDeviationUU(const FScriptedMotionSegment& Segment,
		double FromSeconds, double ToSeconds)
	{
		if (!Segment.bCurved || !Segment.Curve.IsValid() || ToSeconds <= FromSeconds) return 0.0;
		const double from = FMath::Clamp(FromSeconds - Segment.StartSeconds, 0.0, Segment.DurationSeconds);
		const double to = FMath::Clamp(ToSeconds - Segment.StartSeconds, from, Segment.DurationSeconds);
		const double startDistance = Segment.InitialSpeedUU * from + 0.5 * Segment.AccelerationUU * from * from;
		const double endDistance = Segment.InitialSpeedUU * to + 0.5 * Segment.AccelerationUU * to * to;
		const double firstParameter = FindTimeCurveParameter(*Segment.Curve,
			Segment.CurveStartDistanceUU + FMath::Clamp(startDistance, 0.0, Segment.DistanceUU));
		const double lastParameter = FindTimeCurveParameter(*Segment.Curve,
			Segment.CurveStartDistanceUU + FMath::Clamp(endDistance, 0.0, Segment.DistanceUU));
		const FVector original[4] = {Segment.Curve->StartPosition, Segment.Curve->Control1,
			Segment.Curve->Control2, Segment.Curve->EndPosition};
		FVector prefix[4], suffix[4], before[4], interval[4];
		SplitTimeCurveControls(original, lastParameter, prefix, suffix);
		SplitTimeCurveControls(prefix, lastParameter > 0.0 ? firstParameter / lastParameter : 0.0,
			before, interval);
		const double geometryBound = FMath::Max(
			FVector::Dist(interval[1], FMath::Lerp(interval[0], interval[3], 1.0 / 3.0)),
			FVector::Dist(interval[2], FMath::Lerp(interval[0], interval[3], 2.0 / 3.0)));
		const double derivativeBound = 3.0 * FMath::Max(FVector::Dist(interval[0], interval[1]),
			FMath::Max(FVector::Dist(interval[1], interval[2]), FVector::Dist(interval[2], interval[3])));
		const double travel = endDistance - startDistance;
		const double timingBound = travel > UE_DOUBLE_SMALL_NUMBER
			? derivativeBound * FMath::Abs(Segment.AccelerationUU) * FMath::Square(to - from) / (8.0 * travel) : 0.0;
		return geometryBound + timingBound;
	}

	bool BuildTimeCurveGeometry(const TArray<FVector>& Points, double HandleFraction,
		TArray<TSharedPtr<const FTimeCurveGeometry, ESPMode::ThreadSafe>>& OutCurves)
	{
		constexpr int32 ArcIntervals = 96;
		TArray<FVector> tangents;
		TArray<double> handles;
		tangents.SetNum(Points.Num());
		handles.SetNum(Points.Num());
		for (int32 pointIndex = 0; pointIndex < Points.Num(); ++pointIndex)
		{
			const FVector incoming = pointIndex > 0
				? (Points[pointIndex] - Points[pointIndex - 1]).GetSafeNormal() : FVector::ZeroVector;
			const FVector outgoing = pointIndex < Points.Num() - 1
				? (Points[pointIndex + 1] - Points[pointIndex]).GetSafeNormal() : FVector::ZeroVector;
			tangents[pointIndex] = pointIndex == 0 ? outgoing :
				(pointIndex == Points.Num() - 1 ? incoming : (incoming + outgoing).GetSafeNormal());
			if (tangents[pointIndex].IsNearlyZero()) return false;
			const double incomingLength = pointIndex > 0
				? FVector::Dist(Points[pointIndex - 1], Points[pointIndex]) : TNumericLimits<double>::Max();
			const double outgoingLength = pointIndex < Points.Num() - 1
				? FVector::Dist(Points[pointIndex], Points[pointIndex + 1]) : TNumericLimits<double>::Max();
			handles[pointIndex] = HandleFraction * FMath::Min(incomingLength, outgoingLength);
		}
		for (int32 pointIndex = 1; pointIndex < Points.Num(); ++pointIndex)
		{
			TSharedPtr<FTimeCurveGeometry, ESPMode::ThreadSafe> curve = MakeShared<FTimeCurveGeometry, ESPMode::ThreadSafe>();
			curve->StartPosition = Points[pointIndex - 1];
			curve->Control1 = curve->StartPosition + tangents[pointIndex - 1] * handles[pointIndex - 1];
			curve->EndPosition = Points[pointIndex];
			curve->Control2 = curve->EndPosition - tangents[pointIndex] * handles[pointIndex];
			curve->ArcLengths.Reserve(ArcIntervals + 1);
			curve->ArcLengths.Add(0.0);
			FVector previous = curve->StartPosition;
			for (int32 interval = 1; interval <= ArcIntervals; ++interval)
			{
				const double parameter = static_cast<double>(interval) / ArcIntervals;
				const FVector position = EvaluateTimeCurvePoint(*curve, parameter);
				if (EvaluateTimeCurveTangent(*curve, parameter).IsNearlyZero()) return false;
				curve->ArcLengths.Add(curve->ArcLengths.Last() + FVector::Dist(previous, position));
				previous = position;
			}
			if (curve->ArcLengths.Last() <= UE_DOUBLE_SMALL_NUMBER) return false;
			OutCurves.Add(curve);
		}
		return true;
	}

	double GetTimeCurveCruiseSpeed(const TArray<TSharedPtr<const FTimeCurveGeometry, ESPMode::ThreadSafe>>& Curves,
		const FEngineSnapshot& Engine, double RequestedSpeedUU)
	{
		double speed = FMath::Min(RequestedSpeedUU, FMath::Sqrt(2.0 *
			Engine.ThrustAcceleration * MaxForwardFuelPerSecond * Curves[0]->ArcLengths.Last()));
		for (const auto& curve : Curves)
		{
			for (int32 interval = 0; interval < curve->ArcLengths.Num(); ++interval)
			{
				const double parameter = static_cast<double>(interval) / (curve->ArcLengths.Num() - 1);
				const FVector tangent = EvaluateTimeCurveTangent(*curve, parameter);
				const FVector second = ((curve->Control2 - 2.0 * curve->Control1 + curve->StartPosition) *
					(1.0 - parameter) + (curve->EndPosition - 2.0 * curve->Control2 + curve->Control1) * parameter) * 6.0;
				const double curvature = FVector::CrossProduct(tangent, second).Size() /
					FMath::Max(UE_DOUBLE_SMALL_NUMBER, FMath::Pow(tangent.Size(), 3.0));
				if (curvature <= UE_DOUBLE_SMALL_NUMBER) continue;
				speed = FMath::Min(speed, FMath::DegreesToRadians(static_cast<double>(Engine.MaxTurnSpeed)) / curvature);
				speed = FMath::Min(speed, FMath::Sqrt(Engine.ThrustAcceleration * MaxLateralFuelPerSecond / curvature));
			}
		}
		return speed;
	}

	bool AppendTimeCurveSchedule(const FRouteCandidate& Base, const FPlannerContext& Context,
		const TArray<TSharedPtr<const FTimeCurveGeometry, ESPMode::ThreadSafe>>& Curves,
		double SpeedUU, FRouteCandidate& OutCandidate, double& OutRateRatio)
	{
		OutRateRatio = 1.0;
		for (const FScriptedMotionSegment& segment : Base.MotionSegments)
		{
			if (segment.DistanceUU > UE_DOUBLE_SMALL_NUMBER) break;
			OutCandidate.MotionSegments.Add(segment);
		}
		double previousYawRate = 0.0;
		double previousPitchRate = 0.0;
		const double acceleration = Context.Engine.ThrustAcceleration * MaxForwardFuelPerSecond;
		const double accelerationDistance = SpeedUU * SpeedUU / (2.0 * acceleration);
		for (int32 curveIndex = 0; curveIndex < Curves.Num(); ++curveIndex)
		{
			const auto& curve = Curves[curveIndex];
			TArray<double> knots = curve->ArcLengths;
			if (curveIndex == 0 && accelerationDistance > 0.0 && accelerationDistance < knots.Last())
			{
				knots.Add(accelerationDistance);
				knots.Sort();
			}
			for (int32 knotIndex = 1; knotIndex < knots.Num(); ++knotIndex)
			{
				const double startDistance = knots[knotIndex - 1];
				const double endDistance = knots[knotIndex];
				const double distance = endDistance - startDistance;
				if (distance <= UE_DOUBLE_SMALL_NUMBER) continue;
				const double startSpeed = curveIndex == 0
					? FMath::Min(SpeedUU, FMath::Sqrt(2.0 * acceleration * startDistance)) : SpeedUU;
				const double endSpeed = curveIndex == 0
					? FMath::Min(SpeedUU, FMath::Sqrt(2.0 * acceleration * endDistance)) : SpeedUU;
				const double duration = 2.0 * distance / (startSpeed + endSpeed);
				const double startParameter = FindTimeCurveParameter(*curve, startDistance);
				const double endParameter = FindTimeCurveParameter(*curve, endDistance);
				const FVector startTangent = EvaluateTimeCurveTangent(*curve, startParameter).GetSafeNormal();
				const FVector endTangent = EvaluateTimeCurveTangent(*curve, endParameter).GetSafeNormal();
				const FQuat startOrientation = startTangent.Rotation().Quaternion();
				const FQuat endOrientation = endTangent.Rotation().Quaternion();
				const FRotator rotation = (startOrientation.Inverse() * endOrientation).Rotator();
				const double yawRate = rotation.Yaw / duration;
				const double pitchRate = rotation.Pitch / duration;
				const FQuat middleOrientation = FQuat::Slerp(startOrientation, endOrientation, 0.5).GetNormalized();
				const FVector velocityChange = middleOrientation.UnrotateVector(
					endTangent * endSpeed - startTangent * startSpeed);
				FNavigationControl fuel;
				fuel.ForwardFuel = static_cast<float>(MaxForwardFuelPerSecond * duration);
				fuel.RightFuel = static_cast<float>(velocityChange.Y / Context.Engine.ThrustAcceleration);
				fuel.UpFuel = static_cast<float>(velocityChange.Z / Context.Engine.ThrustAcceleration);
				fuel.YawFuel = static_cast<float>((yawRate - previousYawRate +
					FMath::Sign(yawRate) * Context.Engine.TurnDeceleration * duration) / Context.Engine.TurnImpulsePerFuelUnit);
				fuel.PitchFuel = static_cast<float>((pitchRate - previousPitchRate +
					FMath::Sign(pitchRate) * Context.Engine.TurnDeceleration * duration) / Context.Engine.TurnImpulsePerFuelUnit);
				OutRateRatio = FMath::Max(OutRateRatio, FMath::Max(FMath::Abs(yawRate), FMath::Abs(pitchRate)) /
					Context.Engine.MaxTurnSpeed);
				OutRateRatio = FMath::Max(OutRateRatio, FMath::Max(FMath::Abs(static_cast<double>(fuel.RightFuel)),
					FMath::Abs(static_cast<double>(fuel.UpFuel))) / (MaxLateralFuelPerSecond * duration));
				OutRateRatio = FMath::Max(OutRateRatio, FMath::Max(FMath::Abs(static_cast<double>(fuel.YawFuel)),
					FMath::Abs(static_cast<double>(fuel.PitchFuel))) / (MaxTurnFuelPerSecond * duration));
				if (!FMath::IsFinite(duration) || !FMath::IsFinite(GetScriptedFuelKg(fuel))) return false;
				FScriptedMotionSegment& segment = OutCandidate.MotionSegments.AddDefaulted_GetRef();
				segment.StartSeconds = OutCandidate.MotionSegments.Num() > 1
					? OutCandidate.MotionSegments[OutCandidate.MotionSegments.Num() - 2].StartSeconds +
						OutCandidate.MotionSegments[OutCandidate.MotionSegments.Num() - 2].DurationSeconds : 0.0;
				segment.DurationSeconds = duration;
				segment.StartPosition = EvaluateTimeCurvePoint(*curve, startParameter);
				segment.EndPosition = endDistance >= curve->ArcLengths.Last()
					? curve->EndPosition : EvaluateTimeCurvePoint(*curve, endParameter);
				segment.StartOrientation = startOrientation;
				segment.EndOrientation = endOrientation;
				segment.InitialSpeedUU = startSpeed;
				segment.AccelerationUU = (endSpeed - startSpeed) / duration;
				segment.DistanceUU = distance;
				segment.Fuel = fuel;
				segment.Phase = EFlightPhase::Launch;
				segment.NextPointIndex = curveIndex + 1;
				segment.bCurved = true;
				segment.Curve = curve;
				segment.CurveStartDistanceUU = startDistance;
				segment.CurveFuelGroup = curveIndex;
				previousYawRate = yawRate;
				previousPitchRate = pitchRate;
			}
		}
		return OutRateRatio <= 1.0 + KINDA_SMALL_NUMBER;
	}

	void FinalizeTimeCurveSchedule(FRouteCandidate& Candidate)
	{
		int32 group = INDEX_NONE;
		double remainingFuel = 0.0;
		for (int32 segmentIndex = Candidate.MotionSegments.Num() - 1; segmentIndex >= 0; --segmentIndex)
		{
			FScriptedMotionSegment& segment = Candidate.MotionSegments[segmentIndex];
			if (segment.CurveFuelGroup != group) { group = segment.CurveFuelGroup; remainingFuel = 0.0; }
			remainingFuel += GetScriptedFuelKg(segment.Fuel);
			if (segment.bCurved) segment.CurveRemainingFuelKg = remainingFuel;
			Candidate.FuelUsedKg += GetScriptedFuelKg(segment.Fuel);
			Candidate.ForwardFuelUsedKg += segment.Fuel.ForwardFuel;
			Candidate.LateralFuelUsedKg += FMath::Abs(segment.Fuel.RightFuel) + FMath::Abs(segment.Fuel.UpFuel);
			Candidate.TurnFuelUsedKg += FMath::Abs(segment.Fuel.YawFuel) + FMath::Abs(segment.Fuel.PitchFuel);
			if (GetScriptedFuelKg(segment.Fuel) > 0.0) Candidate.BurnDurationSeconds += segment.DurationSeconds;
			else Candidate.CoastDurationSeconds += segment.DurationSeconds;
		}
		const FScriptedMotionSegment& last = Candidate.MotionSegments.Last();
		Candidate.FlightTimeSeconds = last.StartSeconds + last.DurationSeconds;
		for (const FScriptedMotionSegment& segment : Candidate.MotionSegments)
		{
			if (GetScriptedFuelKg(segment.Fuel) <= 0.0) continue;
			FPredictedStep& maneuver = Candidate.Maneuvers.AddDefaulted_GetRef();
			maneuver.TimeSeconds = segment.StartSeconds;
			maneuver.Position = segment.StartPosition;
			maneuver.Control = segment.Fuel;
		}
	}

	bool EvaluateTimeCurveSafety(const FNavigationState& Previous, const FNavigationState& Current,
		const FScriptedMotionSegment& Segment, const FPlannerContext& Context,
		const TArray<FVector>& PreviousBodyPositions, const TArray<FVector>& CurrentBodyPositions,
		double& OutRisk, FRouteCandidate& Candidate, FSimulationFailure* OutFailure)
	{
		int32 blockingBody = INDEX_NONE;
		double physicalGap = 0.0;
		double requiredGap = 0.0;
		bool bSafe = EvaluateSafetyAndRisk(Previous, Current, Context, PreviousBodyPositions,
			CurrentBodyPositions, OutRisk, blockingBody, Candidate.MinimumStarClearanceUU,
			Candidate.MinimumPlanetClearanceUU, physicalGap, requiredGap, Candidate);
		const double seconds = Current.ElapsedSeconds - Previous.ElapsedSeconds;
		const double shipBound = GetTimeCurveDeviationUU(Segment, Previous.ElapsedSeconds, Current.ElapsedSeconds);
		const double shipSpeed = FMath::Max((Previous.EngineVelocity + Previous.GravityVelocity).Size(),
			(Current.EngineVelocity + Current.GravityVelocity).Size());
		for (int32 bodyIndex = 0; bSafe && bodyIndex < Context.Bodies.Num(); ++bodyIndex)
		{
			const FPlannerBody& body = Context.Bodies[bodyIndex];
			const FVector previousRelative = Previous.Position - PreviousBodyPositions[bodyIndex];
			const FVector relativeStep = Current.Position - CurrentBodyPositions[bodyIndex] - previousRelative;
			const double fraction = relativeStep.SizeSquared() > 0.0
				? FMath::Clamp(-FVector::DotProduct(previousRelative, relativeStep) / relativeStep.SizeSquared(), 0.0, 1.0) : 0.0;
			const double closestDistance = (previousRelative + relativeStep * fraction).Size();
			const double orbitBound = body.bPlanet
				? body.Orbit.InitialOffset.Size() * FMath::Square(body.Orbit.AngularSpeedRadiansPerSecond) *
					seconds * seconds / 8.0 : 0.0;
			const double bound = shipBound + orbitBound;
			const double conservativeDistance = FMath::Max(0.0, closestDistance - bound);
			double& clearance = body.bPlanet ? Candidate.MinimumPlanetClearanceUU : Candidate.MinimumStarClearanceUU;
			clearance = FMath::Min(clearance, conservativeDistance - body.RadiusUU);
			const double reserve = CalculateBodyPredictionReserveUU(body, relativeStep.Size() / seconds, seconds);
			if (conservativeDistance < body.RadiusUU + reserve)
			{
				bSafe = false;
				blockingBody = bodyIndex;
				physicalGap = closestDistance - body.RadiusUU;
				requiredGap = reserve + bound;
				break;
			}
			const double closingSpeed = previousRelative.Size() > 0.0
				? FMath::Max(0.0, -FVector::DotProduct(previousRelative.GetSafeNormal(), relativeStep / seconds)) : 0.0;
			OutRisk = FMath::Max(OutRisk, EvaluateRouteBodyRisk(Context, body, Current,
				conservativeDistance, reserve, closingSpeed, shipSpeed, Candidate));
		}
		if (!bSafe && OutFailure != nullptr)
		{
			OutFailure->Reason = Context.Bodies.IsValidIndex(blockingBody) && Context.Bodies[blockingBody].bPlanet
				? ESimulationFailureReason::PlanetCollision : ESimulationFailureReason::StarCollision;
			OutFailure->BodyIndex = blockingBody;
			OutFailure->ElapsedSeconds = Current.ElapsedSeconds;
			OutFailure->TargetDistanceUU = FVector::Dist(Current.Position, Context.Target);
			OutFailure->RemainingFuelKg = Current.FuelKg;
			OutFailure->PhysicalGapUU = physicalGap;
			OutFailure->RequiredGapUU = requiredGap;
		}
		return bSafe;
	}

	bool SampleTimeCurveCandidate(const FPlannerContext& Context, FRouteCandidate& Candidate,
		FSimulationFailure* OutFailure)
	{
		Candidate.AvailableCorrectionFuelKg = FMath::Max(0.0, Context.InitialFuelKg - Candidate.FuelUsedKg);
		Candidate.DesiredCorrectionFuelKg = Candidate.FuelUsedKg * Context.DesiredCorrectionFuelReserveFraction;
		FNavigationState state = Context.InitialState;
		state.GravityVelocity = FVector::ZeroVector;
		state.YawAngularVelocity = 0.0f;
		state.PitchAngularVelocity = 0.0f;
		state.ElapsedSeconds = 0.0;
		int32 segmentIndex = 0;
		const FScriptedPose initial = EvaluateScriptedMotion(Candidate.MotionSegments, 0.0, &segmentIndex);
		state.Position = initial.Position; state.Orientation = initial.Orientation; state.EngineVelocity = initial.Velocity;
		Candidate.PredictedPath.Add(state.Position);
		FPredictedStateSample& initialSample = Candidate.PredictedStates.AddDefaulted_GetRef();
		initialSample.Position = state.Position; initialSample.Velocity = state.EngineVelocity;
		TArray<FVector> currentBodies, nextBodies;
		CachePredictedBodyPositions(Context, 0.0, currentBodies);
		double spentFuel = 0.0;
		double peakRisk = 0.0;
		double accumulatedRisk = 0.0;
		for (int32 sampleIndex = 0; sampleIndex < MaximumPredictionSteps; ++sampleIndex)
		{
			Candidate.PredictionSteps = sampleIndex + 1;
			const FScriptedMotionSegment& segment = Candidate.MotionSegments[segmentIndex];
			const double segmentEnd = segment.StartSeconds + segment.DurationSeconds;
			const double nextSeconds = FMath::Min(segmentEnd,
				state.ElapsedSeconds + GetPredictionStepSeconds(state, Context, currentBodies));
			if (nextSeconds <= state.ElapsedSeconds) break;
			const FNavigationState previous = state;
			FNavigationControl fuel;
			AddScriptedControl(fuel, segment.Fuel, (nextSeconds - state.ElapsedSeconds) / segment.DurationSeconds);
			spentFuel += GetScriptedFuelKg(fuel);
			const FScriptedPose pose = EvaluateScriptedMotion(Candidate.MotionSegments, nextSeconds, &segmentIndex);
			state.Position = pose.Position; state.Orientation = pose.Orientation; state.EngineVelocity = pose.Velocity;
			state.NextPointIndex = pose.NextPointIndex; state.ElapsedSeconds = nextSeconds;
			state.FuelKg = FMath::Max(0.0, Context.InitialFuelKg - spentFuel);
			CachePredictedBodyPositions(Context, nextSeconds, nextBodies);
			double risk = 0.0;
			if (!EvaluateTimeCurveSafety(previous, state, segment, Context, currentBodies, nextBodies,
				risk, Candidate, OutFailure)) return false;
			peakRisk = FMath::Max(peakRisk, risk);
			accumulatedRisk += risk * (nextSeconds - previous.ElapsedSeconds);
			if (FVector::DistSquared(Candidate.PredictedPath.Last(), state.Position) > UE_DOUBLE_SMALL_NUMBER)
				Candidate.PredictedPath.Add(state.Position);
			FPredictedStateSample& sample = Candidate.PredictedStates.AddDefaulted_GetRef();
			sample.TimeSeconds = nextSeconds; sample.Position = state.Position; sample.Velocity = state.EngineVelocity;
			Swap(currentBodies, nextBodies);
			if (nextSeconds >= Candidate.FlightTimeSeconds)
			{
				Candidate.RiskCost = 0.5 * peakRisk + 0.5 * accumulatedRisk / FMath::Max(0.001, nextSeconds);
				return true;
			}
			if (nextSeconds >= segmentEnd) ++segmentIndex;
		}
		if (OutFailure != nullptr) OutFailure->Reason = ESimulationFailureReason::Unverified;
		return false;
	}

	bool BuildTimeCurveCandidate(const FRouteCandidate& Base, const FPlannerContext& Context,
		double HandleFraction, FRouteCandidate& OutCandidate, FSimulationFailure* OutFailure)
	{
		if (OutFailure != nullptr)
		{
			*OutFailure = FSimulationFailure();
			OutFailure->Reason = ESimulationFailureReason::Unverified;
		}
		if (Base.Family != EPlanFamily::ContinuousCorrection || Base.Guidance.Num() < 2 ||
			HandleFraction <= 0.0 || HandleFraction >= 0.5 || !FMath::IsFinite(HandleFraction) ||
			Context.Engine.ThrustAcceleration <= 0.0f || Context.Engine.MaxSpeed <= 0.0f ||
			Context.Engine.TurnImpulsePerFuelUnit <= 0.0f || Context.Engine.MaxTurnSpeed <= 0.0f) return false;
		TArray<FVector> curvePoints = Base.Guidance;
		curvePoints.Last() = Context.Target;
		TArray<TSharedPtr<const FTimeCurveGeometry, ESPMode::ThreadSafe>> curves;
		if (!BuildTimeCurveGeometry(curvePoints, HandleFraction, curves)) return false;
		bool bHasArc = false;
		for (const auto& curve : curves)
		{
			const FVector chord = (curve->EndPosition - curve->StartPosition).GetSafeNormal();
			const FVector firstOffset = curve->Control1 - curve->StartPosition;
			const FVector lastOffset = curve->Control2 - curve->StartPosition;
			if ((firstOffset - chord * FVector::DotProduct(firstOffset, chord)).SizeSquared() > 1.0 ||
				(lastOffset - chord * FVector::DotProduct(lastOffset, chord)).SizeSquared() > 1.0)
			{
				bHasArc = true;
				break;
			}
		}
		if (!bHasArc) return false;
		double speed = GetTimeCurveCruiseSpeed(curves, Context.Engine, Context.Engine.MaxSpeed * Base.CruiseSpeedFraction);
		bool bScheduled = false;
		for (int32 speedAttempt = 0; speedAttempt < 4; ++speedAttempt)
		{
			OutCandidate = FRouteCandidate();
			OutCandidate.Family = EPlanFamily::ContinuousCorrection;
			OutCandidate.Guidance = curvePoints;
			OutCandidate.CruiseSpeedFraction = Base.CruiseSpeedFraction;
			OutCandidate.MacroTurnCount = Base.MacroTurnCount;
			if (!FMath::IsFinite(speed) || speed <= 0.0) return false;
			double rateRatio = 1.0;
			bScheduled = AppendTimeCurveSchedule(Base, Context, curves, speed, OutCandidate, rateRatio);
			if (bScheduled) break;
			if (!FMath::IsFinite(rateRatio) || rateRatio <= 1.0) return false;
			speed /= rateRatio;
		}
		if (!bScheduled || OutCandidate.MotionSegments.IsEmpty()) return false;
		FinalizeTimeCurveSchedule(OutCandidate);
		if (OutCandidate.FuelUsedKg > Context.InitialFuelKg)
		{
			if (OutFailure != nullptr)
			{
				OutFailure->Reason = ESimulationFailureReason::Fuel;
				OutFailure->RequiredFuelKg = OutCandidate.FuelUsedKg;
				OutFailure->RemainingFuelKg = Context.InitialFuelKg;
			}
			return false;
		}
		return SampleTimeCurveCandidate(Context, OutCandidate, OutFailure);
	}

}

namespace
{
	const FRouteCandidate* FindRouteCandidate(const FRouteSearchResults& Results, int32 Index)
	{
		if (Results.Candidates.IsValidIndex(Index)) return &Results.Candidates[Index];
		const int32 curveIndex = Index >= Results.Candidates.Num()
			? Index - Results.Candidates.Num() : INDEX_NONE;
		return Results.TimeCandidates.IsValidIndex(curveIndex)
			? &Results.TimeCandidates[curveIndex] : nullptr;
	}

	void BuildTimeCurveVariants(const FPlannerContext& Context, FRouteSearchResults& Results, bool bLog)
	{
		const double startedSeconds = FPlatformTime::Seconds();
		int32 attempts = 0;
		FSimulationFailureSummary failures;
		int32 rawIndices[3] = {INDEX_NONE, INDEX_NONE, INDEX_NONE};
		double rawCosts[3] = {};
		const double separationUU = Results.Candidates.IsEmpty() ? 0.0 :
			Context.MinimumRouteSeparationFraction * FVector::Dist(
				Results.Candidates[0].Guidance[0], Results.Candidates[0].Guidance.Last());
		SelectDistinctModeCandidates(Results.Candidates, Context.ModeWeights,
			Context.FuelMaxTurns, separationUU, rawIndices, rawCosts);
		for (const FRouteCandidate& baseline : Results.Candidates)
		{
			if (baseline.Family != EPlanFamily::ContinuousCorrection) continue;
			for (const double handleFraction : {0.35, 0.20, 0.10})
			{
				++attempts;
				FRouteCandidate curved;
				FSimulationFailure failure;
				if (!BuildTimeCurveCandidate(baseline, Context, handleFraction, curved, &failure))
				{
					failures.Add(failure);
					continue;
				}
				bool bDistinct = true;
				for (const int32 otherMode : {0, 2})
				{
					const FRouteCandidate* other = FindRouteCandidate(Results, rawIndices[otherMode]);
					if (other == nullptr || ArePredictedPathsDistinct(curved.PredictedPath,
						other->PredictedPath, separationUU)) continue;
					bDistinct = false;
					break;
				}
				Results.TimeCandidates.Add(MoveTemp(curved));
				if (bDistinct) break;
			}
		}
		if (!bLog) return;
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Time curves %d"), Results.TimeCandidates.Num());
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Curve checks %d"), attempts);
		UE_LOG(LogSpaceNavRoute, Display, TEXT("Curve ms %.1f"),
			(FPlatformTime::Seconds() - startedSeconds) * 1000.0);
		failures.Log(TEXT("Curve"), true, Context);
	}

	void SelectTimeCurveCandidate(const FRouteSearchResults& Results,
		const FSpaceNavRouteWeightSet& ModeWeights, double SeparationUU,
		int32* OutIndices, double* OutCosts)
	{
		OutIndices[1] = INDEX_NONE;
		OutCosts[1] = TNumericLimits<double>::Max();
		if (Results.TimeCandidates.IsEmpty()) return;
		double minimumFuel = TNumericLimits<double>::Max();
		double maximumFuel = 0.0;
		double minimumTime = TNumericLimits<double>::Max();
		double maximumTime = 0.0;
		for (const FRouteCandidate& candidate : Results.TimeCandidates)
		{
			minimumFuel = FMath::Min(minimumFuel, candidate.FuelUsedKg);
			maximumFuel = FMath::Max(maximumFuel, candidate.FuelUsedKg);
			minimumTime = FMath::Min(minimumTime, candidate.FlightTimeSeconds);
			maximumTime = FMath::Max(maximumTime, candidate.FlightTimeSeconds);
		}
		FSpaceNavRouteWeightSet weights = ModeWeights;
		const double sum = weights.FuelWeight + weights.TimeWeight + weights.RiskWeight;
		if (!FMath::IsFinite(sum) || sum <= 0.0) return;
		weights.FuelWeight /= sum;
		weights.TimeWeight /= sum;
		weights.RiskWeight /= sum;
		for (int32 curveIndex = 0; curveIndex < Results.TimeCandidates.Num(); ++curveIndex)
		{
			const FRouteCandidate& candidate = Results.TimeCandidates[curveIndex];
			bool bDistinct = true;
			for (const int32 otherMode : {0, 2})
			{
				const FRouteCandidate* other = FindRouteCandidate(Results, OutIndices[otherMode]);
				if (other == nullptr || ArePredictedPathsDistinct(candidate.PredictedPath,
					other->PredictedPath, SeparationUU)) continue;
				bDistinct = false;
				break;
			}
			if (!bDistinct) continue;
			const double cost = GetCandidateCost(candidate, weights,
				minimumFuel, maximumFuel, minimumTime, maximumTime);
			if (cost >= OutCosts[1]) continue;
			OutIndices[1] = Results.Candidates.Num() + curveIndex;
			OutCosts[1] = cost;
		}
	}
}

namespace
{
	void RefreshRouteSafetyContext(FPlannerContext& Context)
	{
		for (FPlannerBody& body : Context.Bodies)
		{
			const AActor* actor = body.Actor.Get();
			if (actor == nullptr) { body.RadiusUU = 0.0; continue; }
			const FVector location = actor->GetActorLocation();
			if (body.bPlanet)
			{
				body.Orbit.InitialOffset = location - body.Orbit.Center;
				body.Orbit.ElapsedSeconds = 0.0;
				body.Orbit.CurrentLocation = location;
			}
			else body.FixedCenter = location;
		}
	}

	void SetScriptedGuardFailure(FSimulationFailure& Failure, ESimulationFailureReason Reason,
		int32 BodyIndex, double Seconds, const FVector& Position, const FPlannerContext& Context,
		double PhysicalGapUU = 0.0, double RequiredGapUU = 0.0)
	{
		Failure.Reason = Reason;
		Failure.BodyIndex = BodyIndex;
		Failure.ElapsedSeconds = Seconds;
		Failure.TargetDistanceUU = FVector::Dist(Position, Context.Target);
		Failure.RemainingFuelKg = Context.InitialState.FuelKg;
		Failure.PhysicalGapUU = PhysicalGapUU;
		Failure.RequiredGapUU = RequiredGapUU;
	}

	double GetScriptedGuardDeviationUU(const FScriptedMotionSegment& Segment,
		double FromSeconds, double ToSeconds)
	{
		if (Segment.bCurved) return GetTimeCurveDeviationUU(Segment, FromSeconds, ToSeconds);
		if (Segment.DistanceUU <= UE_DOUBLE_SMALL_NUMBER) return 0.0;
		return FMath::Abs(Segment.AccelerationUU) * FMath::Square(ToSeconds - FromSeconds) / 8.0;
	}

	bool GetScriptedGuardSpeedUU(const FScriptedMotionSegment& Segment,
		double FromSeconds, double ToSeconds, double& OutSpeedUU)
	{
		const double from = FMath::Clamp(FromSeconds - Segment.StartSeconds, 0.0, Segment.DurationSeconds);
		const double to = FMath::Clamp(ToSeconds - Segment.StartSeconds, from, Segment.DurationSeconds);
		const double scalarSpeed = FMath::Max(0.0, FMath::Max(Segment.InitialSpeedUU + Segment.AccelerationUU * from,
			Segment.InitialSpeedUU + Segment.AccelerationUU * to));
		OutSpeedUU = 0.0;
		if (Segment.DistanceUU <= UE_DOUBLE_SMALL_NUMBER) return true;
		if (!Segment.bCurved)
		{
			OutSpeedUU = scalarSpeed * FVector::Dist(Segment.StartPosition, Segment.EndPosition) / Segment.DistanceUU;
			return FMath::IsFinite(OutSpeedUU);
		}
		if (!Segment.Curve.IsValid() || Segment.Curve->ArcLengths.Num() < 2) return false;
		const double firstParameter = FindTimeCurveParameter(*Segment.Curve, Segment.CurveStartDistanceUU);
		const double lastParameter = FindTimeCurveParameter(*Segment.Curve,
			Segment.CurveStartDistanceUU + Segment.DistanceUU);
		const FVector original[4] = {Segment.Curve->StartPosition, Segment.Curve->Control1,
			Segment.Curve->Control2, Segment.Curve->EndPosition};
		FVector prefix[4], suffix[4], before[4], interval[4];
		SplitTimeCurveControls(original, lastParameter, prefix, suffix);
		SplitTimeCurveControls(prefix, lastParameter > 0.0 ? firstParameter / lastParameter : 0.0, before, interval);
		const double derivativeBound = 3.0 * FMath::Max(FVector::Dist(interval[0], interval[1]),
			FMath::Max(FVector::Dist(interval[1], interval[2]), FVector::Dist(interval[2], interval[3])));
		OutSpeedUU = derivativeBound / Segment.DistanceUU * scalarSpeed;
		return FMath::IsFinite(OutSpeedUU);
	}

	bool BuildScriptedGuardBounds(const FRouteCandidate& Candidate, double FromSeconds, double ToSeconds,
		int32 InitialSegmentIndex, FBox& OutBounds, double& OutSpeedUU, FSimulationFailure& OutFailure,
		const FPlannerContext& Context)
	{
		OutBounds = FBox(ForceInit);
		OutSpeedUU = 0.0;
		int32 cursor = FMath::Clamp(InitialSegmentIndex, 0, Candidate.MotionSegments.Num() - 1);
		const FScriptedPose initial = EvaluateScriptedMotion(Candidate.MotionSegments, FromSeconds, &cursor);
		OutBounds += initial.Position;
		for (int32 segmentIndex = 0; segmentIndex < Candidate.MotionSegments.Num(); ++segmentIndex)
		{
			const FScriptedMotionSegment& segment = Candidate.MotionSegments[segmentIndex];
			const double from = FMath::Max(FromSeconds, segment.StartSeconds);
			const double to = FMath::Min(ToSeconds, segment.StartSeconds + segment.DurationSeconds);
			if (to < from) continue;
			if (!FMath::IsFinite(segment.DurationSeconds) || segment.DurationSeconds <= 0.0)
			{
				SetScriptedGuardFailure(OutFailure, ESimulationFailureReason::InvalidStep, INDEX_NONE,
					from, initial.Position, Context);
				return false;
			}
			cursor = segmentIndex;
			const FVector start = EvaluateScriptedMotion(Candidate.MotionSegments, from, &cursor).Position;
			cursor = segmentIndex;
			const FVector end = EvaluateScriptedMotion(Candidate.MotionSegments, to, &cursor).Position;
			const double deviation = GetScriptedGuardDeviationUU(segment, from, to);
			double speed = 0.0;
			if (start.ContainsNaN() || end.ContainsNaN() || !FMath::IsFinite(deviation) ||
				!GetScriptedGuardSpeedUU(segment, from, to, speed))
			{
				SetScriptedGuardFailure(OutFailure, ESimulationFailureReason::NonFiniteState, INDEX_NONE,
					from, start, Context);
				return false;
			}
			FBox bounds(ForceInit);
			bounds += start; bounds += end;
			OutBounds += bounds.ExpandBy(deviation);
			OutSpeedUU = FMath::Max(OutSpeedUU, speed);
		}
		return !initial.Position.ContainsNaN();
	}

	bool SelectScriptedGuardBodies(const FPlannerContext& Context, const FBox& Bounds, double SpeedUU,
		double HorizonSeconds, double PhaseSlackSeconds, FScriptedGuardScratch& Scratch, FSimulationFailure& OutFailure)
	{
		Scratch.BodyIndices.Reset();
		for (int32 bodyIndex = 0; bodyIndex < Context.Bodies.Num(); ++bodyIndex)
		{
			const FPlannerBody& body = Context.Bodies[bodyIndex];
			if (body.RadiusUU <= 0.0) continue;
			const FVector center = body.LocationAfter(0.0);
			const double orbitSpeed = body.bPlanet
				? body.Orbit.InitialOffset.Size() * FMath::Abs(body.Orbit.AngularSpeedRadiansPerSecond) : 0.0;
			if (center.ContainsNaN() || !FMath::IsFinite(body.RadiusUU) || !FMath::IsFinite(orbitSpeed))
			{
				SetScriptedGuardFailure(OutFailure, ESimulationFailureReason::NonFiniteState,
					bodyIndex, 0.0, Bounds.GetCenter(), Context);
				return false;
			}
			const double reserve = CalculateBodyPredictionReserveUU(body, SpeedUU + orbitSpeed, 0.25);
			const double reach = body.RadiusUU + reserve + orbitSpeed * (HorizonSeconds + PhaseSlackSeconds);
			if (Bounds.ComputeSquaredDistanceToPoint(center) <= reach * reach) Scratch.BodyIndices.Add(bodyIndex);
		}
		Scratch.PreviousBodies.SetNumUninitialized(Scratch.BodyIndices.Num());
		Scratch.CurrentBodies.SetNumUninitialized(Scratch.BodyIndices.Num());
		return true;
	}

	void CacheScriptedGuardBodies(const FPlannerContext& Context, const FScriptedGuardScratch& Scratch,
		double SecondsAhead, TArray<FVector>& OutPositions)
	{
		for (int32 activeIndex = 0; activeIndex < Scratch.BodyIndices.Num(); ++activeIndex)
			OutPositions[activeIndex] = Context.Bodies[Scratch.BodyIndices[activeIndex]].LocationAfter(SecondsAhead);
	}

	bool CheckScriptedGuardInterval(const FScriptedPose& Previous, const FScriptedPose& Current,
		const FScriptedMotionSegment& Segment, double FromSeconds, double ToSeconds,
		const FPlannerContext& Context, double PhaseSlackSeconds, const FScriptedGuardScratch& Scratch,
		FSimulationFailure& OutFailure)
	{
		const double seconds = ToSeconds - FromSeconds;
		const double shipBound = GetScriptedGuardDeviationUU(Segment, FromSeconds, ToSeconds);
		if (Previous.Position.ContainsNaN() || Current.Position.ContainsNaN() || !FMath::IsFinite(shipBound))
		{
			SetScriptedGuardFailure(OutFailure, ESimulationFailureReason::NonFiniteState,
				INDEX_NONE, FromSeconds, Previous.Position, Context);
			return false;
		}
		for (int32 activeIndex = 0; activeIndex < Scratch.BodyIndices.Num(); ++activeIndex)
		{
			const int32 bodyIndex = Scratch.BodyIndices[activeIndex];
			const FPlannerBody& body = Context.Bodies[bodyIndex];
			const FVector previousRelative = Previous.Position - Scratch.PreviousBodies[activeIndex];
			const FVector relativeStep = Current.Position - Scratch.CurrentBodies[activeIndex] - previousRelative;
			const double fraction = relativeStep.SizeSquared() > 0.0
				? FMath::Clamp(-FVector::DotProduct(previousRelative, relativeStep) / relativeStep.SizeSquared(), 0.0, 1.0) : 0.0;
			const double distance = (previousRelative + relativeStep * fraction).Size();
			const double orbitSpeed = body.bPlanet
				? body.Orbit.InitialOffset.Size() * FMath::Abs(body.Orbit.AngularSpeedRadiansPerSecond) : 0.0;
			const double orbitBound = body.bPlanet ? body.Orbit.InitialOffset.Size() *
				FMath::Square(body.Orbit.AngularSpeedRadiansPerSecond) * seconds * seconds / 8.0 : 0.0;
			const double relativeSpeed = seconds > 0.0 ? relativeStep.Size() / seconds : 0.0;
			const double reserve = CalculateBodyPredictionReserveUU(body, relativeSpeed, seconds) +
				shipBound + orbitBound + orbitSpeed * PhaseSlackSeconds;
			if (!FMath::IsFinite(distance) || !FMath::IsFinite(reserve) || distance < body.RadiusUU + reserve)
			{
				SetScriptedGuardFailure(OutFailure, !FMath::IsFinite(distance) || !FMath::IsFinite(reserve)
					? ESimulationFailureReason::NonFiniteState : (body.bPlanet
						? ESimulationFailureReason::PlanetCollision : ESimulationFailureReason::StarCollision),
					bodyIndex, FromSeconds + seconds * fraction, FMath::Lerp(Previous.Position, Current.Position, fraction),
					Context, distance - body.RadiusUU, reserve);
				return false;
			}
		}
		return true;
	}

	bool SampleScriptedGuardWindow(const FRouteCandidate& Candidate, double FromSeconds, double ToSeconds,
		const FPlannerContext& Context, double PhaseSlackSeconds, FScriptedGuardScratch& Scratch,
		FSimulationFailure& OutFailure, int32 InitialSegmentIndex)
	{
		int32 segmentIndex = FMath::Clamp(InitialSegmentIndex, 0, Candidate.MotionSegments.Num() - 1);
		FScriptedPose pose = EvaluateScriptedMotion(Candidate.MotionSegments, FromSeconds, &segmentIndex);
		CacheScriptedGuardBodies(Context, Scratch, 0.0, Scratch.PreviousBodies);
		CacheScriptedGuardBodies(Context, Scratch, 0.0, Scratch.CurrentBodies);
		if (!CheckScriptedGuardInterval(pose, pose, Candidate.MotionSegments[segmentIndex],
			FromSeconds, FromSeconds, Context, PhaseSlackSeconds, Scratch, OutFailure)) return false;
		double elapsed = FromSeconds;
		for (int32 stepIndex = 0; elapsed < ToSeconds && stepIndex < MaximumPredictionSteps; ++stepIndex)
		{
			while (segmentIndex < Candidate.MotionSegments.Num() - 1 && elapsed >=
				Candidate.MotionSegments[segmentIndex].StartSeconds + Candidate.MotionSegments[segmentIndex].DurationSeconds)
				++segmentIndex;
			const FScriptedMotionSegment& segment = Candidate.MotionSegments[segmentIndex];
			double speed = 0.0;
			if (!GetScriptedGuardSpeedUU(segment, elapsed,
				FMath::Min(ToSeconds, segment.StartSeconds + segment.DurationSeconds), speed)) break;
			double stepSeconds = 0.25;
			for (int32 activeIndex = 0; activeIndex < Scratch.BodyIndices.Num(); ++activeIndex)
			{
				const FPlannerBody& body = Context.Bodies[Scratch.BodyIndices[activeIndex]];
				const double bodySpeed = body.bPlanet
					? body.Orbit.InitialOffset.Size() * FMath::Abs(body.Orbit.AngularSpeedRadiansPerSecond) : 0.0;
				const double clearance = FMath::Max(0.0,
					FVector::Dist(pose.Position, Scratch.PreviousBodies[activeIndex]) - body.RadiusUU);
				stepSeconds = FMath::Min(stepSeconds, FMath::Clamp(
					clearance / FMath::Max(100.0, speed + bodySpeed) * 0.2, 0.025, 0.25));
			}
			const double nextSeconds = FMath::Min(ToSeconds,
				FMath::Min(segment.StartSeconds + segment.DurationSeconds, elapsed + stepSeconds));
			if (nextSeconds <= elapsed) break;
			const FScriptedPose next = EvaluateScriptedMotion(Candidate.MotionSegments, nextSeconds, &segmentIndex);
			CacheScriptedGuardBodies(Context, Scratch, nextSeconds - FromSeconds, Scratch.CurrentBodies);
			if (!CheckScriptedGuardInterval(pose, next, segment, elapsed, nextSeconds,
				Context, PhaseSlackSeconds, Scratch, OutFailure)) return false;
			Swap(Scratch.PreviousBodies, Scratch.CurrentBodies);
			pose = next; elapsed = nextSeconds;
		}
		if (elapsed >= ToSeconds) return true;
		SetScriptedGuardFailure(OutFailure, ESimulationFailureReason::Unverified,
			INDEX_NONE, elapsed, pose.Position, Context);
		return false;
	}

	bool ValidateScriptedWindow(const FRouteCandidate& Candidate, double FromSeconds, double ToSeconds,
		const FPlannerContext& Context, double PhaseSlackSeconds, FScriptedGuardScratch& Scratch,
		FSimulationFailure& OutFailure, int32 InitialSegmentIndex)
	{
		OutFailure = FSimulationFailure();
		if (Candidate.MotionSegments.IsEmpty() || !FMath::IsFinite(FromSeconds) || !FMath::IsFinite(ToSeconds) ||
			!FMath::IsFinite(PhaseSlackSeconds) || PhaseSlackSeconds < 0.0 || FromSeconds < 0.0 ||
			ToSeconds < FromSeconds || FromSeconds > Candidate.FlightTimeSeconds) return false;
		ToSeconds = FMath::Min(ToSeconds, Candidate.FlightTimeSeconds);
		FBox bounds(ForceInit);
		double speed = 0.0;
		if (!BuildScriptedGuardBounds(Candidate, FromSeconds, ToSeconds, InitialSegmentIndex,
			bounds, speed, OutFailure, Context)) return false;
		if (!SelectScriptedGuardBodies(Context, bounds, speed, ToSeconds - FromSeconds,
			PhaseSlackSeconds, Scratch, OutFailure)) return false;
		return SampleScriptedGuardWindow(Candidate, FromSeconds, ToSeconds, Context,
			PhaseSlackSeconds, Scratch, OutFailure, InitialSegmentIndex);
	}

	void LogRouteSafetyResult(const TCHAR* Stage, const FSimulationFailure* Failure,
		double Milliseconds, bool bLog)
	{
		if (Failure == nullptr)
		{
			if (bLog) UE_LOG(LogSpaceNavRoute, Display, TEXT("%s safe %.3gms"), Stage, Milliseconds);
			return;
		}
		const TCHAR* reason = Failure->Reason == ESimulationFailureReason::PlanetCollision ? TEXT("planet") :
			(Failure->Reason == ESimulationFailureReason::StarCollision ? TEXT("star") :
				(Failure->Reason == ESimulationFailureReason::Unverified ? TEXT("unverified") : TEXT("invalid")));
		UE_LOG(LogSpaceNavRoute, Warning, TEXT("%s unsafe %s"), Stage, reason);
		UE_LOG(LogSpaceNavRoute, Warning, TEXT("Body %d"), Failure->BodyIndex);
		UE_LOG(LogSpaceNavRoute, Warning, TEXT("Time s %.3g"), Failure->ElapsedSeconds);
		UE_LOG(LogSpaceNavRoute, Warning, TEXT("Gap m %.3g"), Failure->PhysicalGapUU / UnrealUnitsPerMeter);
		UE_LOG(LogSpaceNavRoute, Warning, TEXT("Need m %.3g"), Failure->RequiredGapUU / UnrealUnitsPerMeter);
	}
}
