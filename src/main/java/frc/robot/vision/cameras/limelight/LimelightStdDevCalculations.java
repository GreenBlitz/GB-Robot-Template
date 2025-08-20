package frc.robot.vision.cameras.limelight;

import frc.robot.vision.StdDevs;

import java.util.function.Supplier;

public class LimelightStdDevCalculations {

	public static final StdDevs DEFAULT_STD_DEVS = new StdDevs(0.0003, 0.0003, 0.003);

	public static Supplier<StdDevs> getMT1StdDevsCalculation(Limelight limelight, StdDevs minStdDevs, StdDevs stdDevFactors) {
		return () -> averageTagDistanceParabola(limelight.getMT1RawData().avgTagDist, minStdDevs, stdDevFactors);
	}

	public static Supplier<StdDevs> getMT2StdDevsCalculation(Limelight limelight, StdDevs minStdDevs, StdDevs stdDevFactors) {
		return () -> averageTagDistanceParabola(limelight.getMT2RawData().avgTagDist, minStdDevs, stdDevFactors);
	}

	private static StdDevs averageTagDistanceParabola(
		double averageTagDistance,
		StdDevs minStandardDeviations,
		StdDevs standardDeviationFactors
	) {
		double averageTagDistanceSquared = Math.pow(averageTagDistance, 2);
		return new StdDevs(
			Math.max(minStandardDeviations.x(), standardDeviationFactors.x() * averageTagDistanceSquared),
			Math.max(minStandardDeviations.y(), standardDeviationFactors.y() * averageTagDistanceSquared),
			Math.max(minStandardDeviations.rotation(), standardDeviationFactors.rotation() * averageTagDistanceSquared)
		);
	}

}
