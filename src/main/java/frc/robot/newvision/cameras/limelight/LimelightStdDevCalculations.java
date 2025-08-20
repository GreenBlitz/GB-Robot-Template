package frc.robot.newvision.cameras.limelight;

import java.util.function.Supplier;

public class LimelightStdDevCalculations {

	public static final double[] DEFAULT_STD_DEVS = new double[] {0.0003, 0.0003, 0.003};

	public static Supplier<double[]> getMT1StdDevsCalculation(Limelight limelight, double[] minStdDevs, double[] stdDevFactors) {
		return () -> averageTagDistanceParabola(limelight.getMT1RawData().avgTagDist, minStdDevs, stdDevFactors);
	}

	public static Supplier<double[]> getMT2StdDevsCalculation(Limelight limelight, double[] minStdDevs, double[] stdDevFactors) {
		return () -> averageTagDistanceParabola(limelight.getMT2RawData().avgTagDist, minStdDevs, stdDevFactors);
	}

	private static double[] averageTagDistanceParabola(
		double averageTagDistance,
		double[] minStandardDeviations,
		double[] standardDeviationFactors
	) {
		double averageTagDistanceSquared = Math.pow(averageTagDistance, 2);
		return new double[] {
			Math.max(minStandardDeviations[0], standardDeviationFactors[0] * averageTagDistanceSquared),
			Math.max(minStandardDeviations[1], standardDeviationFactors[1] * averageTagDistanceSquared),
			Math.max(minStandardDeviations[2], standardDeviationFactors[2] * averageTagDistanceSquared)};
	}

}
