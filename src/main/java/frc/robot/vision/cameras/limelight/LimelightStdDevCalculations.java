package frc.robot.vision.cameras.limelight;

import frc.utils.math.StandardDeviations2D;

import java.util.function.Supplier;

public class LimelightStdDevCalculations {

	public static final StandardDeviations2D DEFAULT_STD_DEVS = new StandardDeviations2D(0.0003, 0.0003, 0.003);

	public static Supplier<StandardDeviations2D> getMT1StdDevsCalculation(
		Limelight limelight,
		StandardDeviations2D stdDevFactorials,
		StandardDeviations2D stdDevFactors
	) {
		return () -> exponentialTagDistanceDividedByVisibleTags(
			limelight.getMT1RawData().tagCount(),
			limelight.getMT1RawData().avgTagDist(),
			stdDevFactorials,
			stdDevFactors
		);
	}

	public static Supplier<StandardDeviations2D> getMT2StdDevsCalculation(
		Limelight limelight,
		StandardDeviations2D minStdDevs,
		StandardDeviations2D stdDevFactors
	) {
		return () -> averageTagDistanceParabola(limelight.getMT2RawData().avgTagDist(), minStdDevs, stdDevFactors);
	}

	private static StandardDeviations2D averageTagDistanceParabola(
		double averageTagDistance,
		StandardDeviations2D minStandardDeviations,
		StandardDeviations2D standardDeviationFactors
	) {
		double averageTagDistanceSquared = Math.pow(averageTagDistance, 2);
		return new StandardDeviations2D(
			Math.max(minStandardDeviations.xStandardDeviations(), standardDeviationFactors.xStandardDeviations() * averageTagDistanceSquared),
			Math.max(minStandardDeviations.yStandardDeviations(), standardDeviationFactors.yStandardDeviations() * averageTagDistanceSquared),
			Math.max(
				minStandardDeviations.angleStandardDeviations(),
				standardDeviationFactors.angleStandardDeviations() * averageTagDistanceSquared
			)
		);
	}

	private static StandardDeviations2D averageTagDistanceDividedByVisibleTagParabola(
		double numberOfVisibleTags,
		double averageTagDistance,
		StandardDeviations2D minStandardDeviations,
		StandardDeviations2D standardDeviationFactors
	) {
		double averageTagDistanceSquaredDividedByVisibleTags = Math.pow(averageTagDistance, 2) / numberOfVisibleTags;
		return new StandardDeviations2D(
			Math.max(
				minStandardDeviations.xStandardDeviations(),
				standardDeviationFactors.xStandardDeviations() * averageTagDistanceSquaredDividedByVisibleTags
			),
			Math.max(
				minStandardDeviations.yStandardDeviations(),
				standardDeviationFactors.yStandardDeviations() * averageTagDistanceSquaredDividedByVisibleTags
			),
			Math.max(
				minStandardDeviations.angleStandardDeviations(),
				standardDeviationFactors.angleStandardDeviations() * averageTagDistanceSquaredDividedByVisibleTags
			)
		);
	}

	private static StandardDeviations2D exponentialTagDistanceDividedByVisibleTags(
		double numberOfVisibleTags,
		double averageTagDistance,
		StandardDeviations2D standardDeviationFactorials,
		StandardDeviations2D standardDeviationFactors
	) {
		return new StandardDeviations2D(
			Math.exp(standardDeviationFactorials.xStandardDeviations() * averageTagDistance)
				* standardDeviationFactors.xStandardDeviations()
				/ numberOfVisibleTags,
			Math.exp(standardDeviationFactorials.yStandardDeviations() * averageTagDistance)
				* standardDeviationFactors.yStandardDeviations()
				/ numberOfVisibleTags,
			Math.exp(standardDeviationFactorials.angleStandardDeviations() * averageTagDistance)
				* standardDeviationFactors.angleStandardDeviations()
				/ numberOfVisibleTags
		);
	}

}
