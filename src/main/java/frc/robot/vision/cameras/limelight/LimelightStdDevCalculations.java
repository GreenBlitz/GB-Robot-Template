package frc.robot.vision.cameras.limelight;

import frc.utils.math.StandardDeviations2D;

import java.util.function.Supplier;

public class LimelightStdDevCalculations {

	public static final StandardDeviations2D DEFAULT_STD_DEVS = new StandardDeviations2D(0.0003, 0.0003, 0.003);

	public static Supplier<StandardDeviations2D> getMT1StdDevsCalculation(
		Limelight limelight,
		StandardDeviations2D stdDevExponents,
		StandardDeviations2D stdDevFactors,
		StandardDeviations2D stdDevAdditions
	) {
		return () -> exponentialTagDistanceDividedByVisibleTags(
			limelight.getMT1RawData().tagCount(),
			limelight.getMT1PrimaryTagPoseInCameraSpace().getTranslation().getNorm(),
			stdDevExponents,
			stdDevFactors,
			stdDevAdditions
		);
	}

	public static Supplier<StandardDeviations2D> getMT2StdDevsCalculation(
		Limelight limelight,
		StandardDeviations2D minStdDevs,
		StandardDeviations2D stdDevFactors
	) {
		return () -> primaryTagDistanceParabola(
			limelight.getMT2PrimaryTagPoseInCameraSpace().getTranslation().getNorm(),
			minStdDevs,
			stdDevFactors
		);
	}

	private static StandardDeviations2D primaryTagDistanceParabola(
		double primaryTagDistanceFromCamera,
		StandardDeviations2D minStandardDeviations,
		StandardDeviations2D standardDeviationFactors
	) {
		double primaryTagDistanceSquared = Math.pow(primaryTagDistanceFromCamera, 2);
		return new StandardDeviations2D(
			Math.max(minStandardDeviations.xStandardDeviations(), standardDeviationFactors.xStandardDeviations() * primaryTagDistanceSquared),
			Math.max(minStandardDeviations.yStandardDeviations(), standardDeviationFactors.yStandardDeviations() * primaryTagDistanceSquared),
			Math.max(
				minStandardDeviations.angleStandardDeviations(),
				standardDeviationFactors.angleStandardDeviations() * primaryTagDistanceSquared
			)
		);
	}

	private static StandardDeviations2D exponentialTagDistanceDividedByVisibleTags(
		double numberOfVisibleTags,
		double primaryTagDistanceFromCamera,
		StandardDeviations2D standardDeviationExponents,
		StandardDeviations2D standardDeviationFactors,
		StandardDeviations2D standardDeviationAdditions
	) {
		return new StandardDeviations2D(
			Math.exp(standardDeviationExponents.xStandardDeviations() * primaryTagDistanceFromCamera)
				* standardDeviationFactors.xStandardDeviations()
				/ numberOfVisibleTags
				+ standardDeviationAdditions.xStandardDeviations(),
			Math.exp(standardDeviationExponents.yStandardDeviations() * primaryTagDistanceFromCamera)
				* standardDeviationFactors.yStandardDeviations()
				/ numberOfVisibleTags
				+ standardDeviationAdditions.yStandardDeviations(),
			Math.exp(standardDeviationExponents.angleStandardDeviations() * primaryTagDistanceFromCamera)
				* standardDeviationFactors.angleStandardDeviations()
				/ numberOfVisibleTags
				+ standardDeviationAdditions.angleStandardDeviations()
		);
	}

}
