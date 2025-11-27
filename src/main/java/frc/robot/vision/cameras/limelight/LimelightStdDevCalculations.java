package frc.robot.vision.cameras.limelight;

import frc.utils.math.StandardDeviations2D;

import java.util.function.Supplier;

public class LimelightStdDevCalculations {

	public static final StandardDeviations2D DEFAULT_STD_DEVS = new StandardDeviations2D(0.0003, 0.0003, 0.003);

	public static Supplier<StandardDeviations2D> getMT1StdDevsCalculation(
		Limelight limelight,
		StandardDeviations2D tagDistanceFactors,
		StandardDeviations2D standardDeviationFactors,
		StandardDeviations2D visibleTagsExponents,
		StandardDeviations2D standardDeviationAdditions
	) {
		return () -> byExponentialTagDistanceDividedByPoweredVisibleTags(
			limelight.getMT1RawData().tagCount(),
			limelight.getMT1PrimaryTagPoseInCameraSpace().getTranslation().getNorm(),
			tagDistanceFactors,
			standardDeviationFactors,
			visibleTagsExponents,
			standardDeviationAdditions
		);
	}

	public static Supplier<StandardDeviations2D> getMT2StdDevsCalculation(
		Limelight limelight,
		StandardDeviations2D minStdDevs,
		StandardDeviations2D stdDevFactors
	) {
		return () -> byPrimaryTagDistanceSquared(
			limelight.getMT2PrimaryTagPoseInCameraSpace().getTranslation().getNorm(),
			minStdDevs,
			stdDevFactors
		);
	}

	private static StandardDeviations2D byPrimaryTagDistanceSquared(
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

	private static StandardDeviations2D byExponentialTagDistanceDividedByPoweredVisibleTags(
		double numberOfVisibleTags,
		double primaryTagDistanceFromCamera,
		StandardDeviations2D tagDistanceFactors,
		StandardDeviations2D standardDeviationFactors,
		StandardDeviations2D visibleTagsExponents,
		StandardDeviations2D standardDeviationAdditions
	) {
		return new StandardDeviations2D(
			Math.exp(tagDistanceFactors.xStandardDeviations() * primaryTagDistanceFromCamera)
				* standardDeviationFactors.xStandardDeviations()
				/ Math.pow(numberOfVisibleTags, visibleTagsExponents.xStandardDeviations())
				+ standardDeviationAdditions.xStandardDeviations(),
			Math.exp(tagDistanceFactors.yStandardDeviations() * primaryTagDistanceFromCamera)
				* standardDeviationFactors.yStandardDeviations()
				/ Math.pow(numberOfVisibleTags, visibleTagsExponents.yStandardDeviations())
				+ standardDeviationAdditions.yStandardDeviations(),
			Math.exp(tagDistanceFactors.angleStandardDeviations() * primaryTagDistanceFromCamera)
				* standardDeviationFactors.angleStandardDeviations()
				/ Math.pow(numberOfVisibleTags, visibleTagsExponents.angleStandardDeviations())
				+ standardDeviationAdditions.angleStandardDeviations()
		);
	}

}
