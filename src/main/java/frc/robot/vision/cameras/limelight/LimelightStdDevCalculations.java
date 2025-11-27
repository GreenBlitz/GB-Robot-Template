package frc.robot.vision.cameras.limelight;

import frc.utils.math.StandardDeviations2D;

import java.util.function.Supplier;

public class LimelightStdDevCalculations {

	public static final StandardDeviations2D DEFAULT_STD_DEVS = new StandardDeviations2D(0.0003, 0.0003, 0.003);

	public static Supplier<StandardDeviations2D> getMT1StdDevsCalculation(
		Limelight limelight,
		StandardDeviations2D tagDistanceFactors,
		StandardDeviations2D stdDevFactors,
		StandardDeviations2D visibleTagsExponents,
		StandardDeviations2D stdDevAdditions
	) {
		return () -> byExponentialTagDistanceDividedByPoweredVisibleTags(
			limelight.getMT1RawData().tagCount(),
			limelight.getMT1PrimaryTagPoseInCameraSpace().getTranslation().getNorm(),
			tagDistanceFactors,
			stdDevFactors,
			visibleTagsExponents,
			stdDevAdditions
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
		StandardDeviations2D minStdDevs,
		StandardDeviations2D stdDevFactors
	) {
		double primaryTagDistanceSquared = Math.pow(primaryTagDistanceFromCamera, 2);
		return new StandardDeviations2D(
			Math.max(minStdDevs.xStandardDeviations(), stdDevFactors.xStandardDeviations() * primaryTagDistanceSquared),
			Math.max(minStdDevs.yStandardDeviations(), stdDevFactors.yStandardDeviations() * primaryTagDistanceSquared),
			Math.max(
				minStdDevs.angleStandardDeviations(),
				stdDevFactors.angleStandardDeviations() * primaryTagDistanceSquared
			)
		);
	}

	private static StandardDeviations2D byExponentialTagDistanceDividedByPoweredVisibleTags(
		double numberOfVisibleTags,
		double primaryTagDistanceFromCamera,
		StandardDeviations2D tagDistanceFactors,
		StandardDeviations2D stdDevFactors,
		StandardDeviations2D visibleTagsExponents,
		StandardDeviations2D stdDevAdditions
	) {
		return new StandardDeviations2D(
			Math.exp(tagDistanceFactors.xStandardDeviations() * primaryTagDistanceFromCamera)
				* stdDevFactors.xStandardDeviations()
				/ Math.pow(numberOfVisibleTags, visibleTagsExponents.xStandardDeviations())
				+ stdDevAdditions.xStandardDeviations(),
			Math.exp(tagDistanceFactors.yStandardDeviations() * primaryTagDistanceFromCamera)
				* stdDevFactors.yStandardDeviations()
				/ Math.pow(numberOfVisibleTags, visibleTagsExponents.yStandardDeviations())
				+ stdDevAdditions.yStandardDeviations(),
			Math.exp(tagDistanceFactors.angleStandardDeviations() * primaryTagDistanceFromCamera)
				* stdDevFactors.angleStandardDeviations()
				/ Math.pow(numberOfVisibleTags, visibleTagsExponents.angleStandardDeviations())
				+ stdDevAdditions.angleStandardDeviations()
		);
	}

}
