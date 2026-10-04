#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "SpaceNavRouteOptimizationDataAsset.generated.h"

USTRUCT(BlueprintType)
struct FSpaceNavRouteWeightSet
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weights", meta = (ClampMin = "0.0", ClampMax = "1.0", ToolTip = "How much fuel use matters when choosing a route. Higher values favor routes that use less fuel. 0 ignores fuel use."))
	double FuelWeight = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weights", meta = (ClampMin = "0.0", ClampMax = "1.0", ToolTip = "How much flight time matters when choosing a route. Higher values favor shorter flight times. 0 ignores flight time."))
	double TimeWeight = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weights", meta = (ClampMin = "0.0", ClampMax = "1.0", ToolTip = "How much risk matters when choosing a route. Higher values favor routes with less risk. 0 ignores the risk score. Routes that fail the safety check are still blocked."))
	double RiskWeight = 0.0;
};

UCLASS(BlueprintType)
class SPACENAVSIM_API USpaceNavRouteOptimizationDataAsset : public UDataAsset
{
	GENERATED_BODY()

public:
	USpaceNavRouteOptimizationDataAsset()
	{
		FuelEfficient.FuelWeight = 1.0;
		TimeEfficient.TimeWeight = 1.0;
		Balanced.FuelWeight = 0.6;
		Balanced.TimeWeight = 0.3;
		Balanced.RiskWeight = 0.1;
	}

	UPROPERTY(EditAnywhere, Category = "Settings|Fuel Efficient", meta = (ToolTip = "Set how much fuel use, flight time, and risk matter when choosing the fuel-saving route."))
	FSpaceNavRouteWeightSet FuelEfficient;

	UPROPERTY(EditAnywhere, Category = "Settings|Time Efficient", meta = (ToolTip = "Set how much fuel use, flight time, and risk matter when choosing the fast route."))
	FSpaceNavRouteWeightSet TimeEfficient;

	UPROPERTY(EditAnywhere, Category = "Settings|Balanced", meta = (ToolTip = "Set how much fuel use, flight time, and risk matter when choosing the balanced route."))
	FSpaceNavRouteWeightSet Balanced;

	UPROPERTY(EditAnywhere, Category = "Settings|Prediction", meta = (ClampMin = "0", ClampMax = "2", ToolTip = "How many corners the fuel-saving route may have. 0 = no corners, 1 = one corner, 2 = two corners."))
	int32 FuelMaxTurns = 2;

	UPROPERTY(EditAnywhere, Category = "Settings|Prediction", meta = (ClampMin = "0.01", ClampMax = "1.0", ToolTip = "Speeds to try, based on the ship's top speed. 0.5 = 50%, 0.75 = 75%, 1.0 = 100%. The final speed may be lower."))
	TArray<double> CruiseSpeedFractions = {0.5, 0.75, 1.0};

	UPROPERTY(EditAnywhere, Category = "Settings|Prediction", meta = (ClampMin = "1", ToolTip = "Maximum number of early route options to try. More options can make the search slower."))
	int32 MaxGuideCandidates = 200;

	UPROPERTY(EditAnywhere, Category = "Settings|Prediction", meta = (ClampMin = "1", ToolTip = "Maximum number of detailed checks for basic route options. More checks can take more time. Curved routes have extra checks."))
	int32 MaxDetailedCandidates = 32;

	UPROPERTY(EditAnywhere, Category = "Settings|Prediction", meta = (ClampMin = "0.0", ClampMax = "1.0", ToolTip = "How much the route shapes must differ. 0.02 means 2% of the straight distance from A to B. Higher values ask for bigger differences."))
	double MinimumRouteSeparationFraction = 0.02;

	UPROPERTY(EditAnywhere, Category = "Settings|Risk", meta = (ClampMin = "0.0", DisplayName = "Critical Body Clearance (radius fraction)", ToolTip = "Extra distance above the body surface that every route must keep. 0.6 means 1.6 radii from the center. Closer routes are blocked, even when Risk Weight is 0."))
	double CriticalBodyClearanceRadiusFraction = 0.6;

	UPROPERTY(EditAnywhere, Category = "Settings|Risk", meta = (DisplayName = "Ship to Planet Speed Factor", ToolTip = "Wanted ship speed near a planet, compared with the planet's speed. 2 means twice the planet's speed. Below this speed, risk can rise. 0 or a negative value turns off this extra speed check. This does not block a route."))
	double ShipToPlanetSpeedFactor = 2.0;

	UPROPERTY(EditAnywhere, Category = "Settings|Risk", meta = (ClampMin = "0.0", ClampMax = "1.0", DisplayName = "Expected Maneuver Error (0.01 = 1%)", ToolTip = "Possible error in the ship's movement. Enter 0.01 for 1%, 0.2 for 20%, or 0.5 for 50%. More error can raise risk near a body and increase the fuel needed for corrections. Used only for the risk estimate."))
	double ManeuverSpeedErrorFraction = 0.01;

	UPROPERTY(EditAnywhere, Category = "Settings|Risk", meta = (ClampMin = "0.0", ClampMax = "100.0", DisplayName = "Desired Correction Fuel Reserve (%)", ToolTip = "Extra fuel wanted for corrections, as a percent of route fuel. 20 means 20% extra. Too little extra fuel raises risk but does not block the route. The estimate does not spend this fuel."))
	double DesiredCorrectionFuelReservePercent = 20.0;
};
