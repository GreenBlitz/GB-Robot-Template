package frc.robot.vision.limelights;

import edu.wpi.first.math.geometry.*;

public class LimelightFilters {

	protected static boolean keepLimelightData(
		LimelightRawData limelightRawData,
		Pose2d currentEstimatedPose,
		double aprilTagHeightMeters,
		LimelightFiltersTolerances tolerances
	) {
		return LimelightFilters.isAprilTagInProperHeight(limelightRawData, tolerances.aprilTagHeightToleranceMeters(), aprilTagHeightMeters)
			&& LimelightFilters.isLimelightOutputInTolerance(
				limelightRawData,
				currentEstimatedPose,
				tolerances.normalizedPositionTolerance(),
				tolerances.normalizedRotationTolerance()
			)
			&& LimelightFilters.isRollInTolerance(limelightRawData, tolerances.rollTolerance())
			&& LimelightFilters.isPitchInTolerance(limelightRawData, tolerances.pitchTolerance())
			&& LimelightFilters.isRobotOnGround(limelightRawData, tolerances.robotToGroundToleranceMeters());
	}

	protected static boolean isLimelightOutputInTolerance(
		LimelightRawData limelightRawData,
		Pose2d estimatedPose,
		double normalizedPositionTolerance,
		double normalizedRotationTolerance
	) {
		Pose3d limelightPosition = limelightRawData.estimatedPose();
		Pose3d estimatedPose3d = new Pose3d(
			estimatedPose.getX(),
			estimatedPose.getY(),
			0,
			new Rotation3d(0, 0, estimatedPose.getRotation().getRadians())
		);
		Transform3d transformDifference = limelightPosition.minus(estimatedPose3d);
		Rotation3d rotationDifference = limelightPosition.getRotation().minus(estimatedPose3d.getRotation());
		return transformDifference.getTranslation().getNorm() <= normalizedPositionTolerance
			&& getRotationNorm(rotationDifference) <= normalizedRotationTolerance;
	}

	private static double getRotationNorm(Rotation3d angle) {
		return Math.sqrt(Math.pow(angle.getX(), 2) + Math.pow(angle.getY(), 2) + Math.pow(angle.getZ(), 2));
	}

	protected static boolean isPitchInTolerance(LimelightRawData limelightRawData, Rotation2d pitchTolerance) {
		return Math.abs(limelightRawData.estimatedPose().getRotation().getY()) <= pitchTolerance.getRadians();
	}

	protected static boolean isRollInTolerance(LimelightRawData limelightRawData, Rotation2d rollTolerance) {
		return Math.abs(limelightRawData.estimatedPose().getRotation().getX()) <= rollTolerance.getRadians();
	}

	protected static boolean
		isAprilTagInProperHeight(LimelightRawData limelightRawData, double aprilTagHeightToleranceMeters, double aprilTagHeightMeters) {
		double aprilTagHeightConfidence = Math.abs(limelightRawData.aprilTagHeight() - aprilTagHeightMeters);
		return aprilTagHeightConfidence <= aprilTagHeightToleranceMeters;
	}

	protected static boolean isRobotOnGround(LimelightRawData limelightRawData, double robotToGroundToleranceMeters) {
		return limelightRawData.estimatedPose().getZ() <= robotToGroundToleranceMeters;
	}

}
