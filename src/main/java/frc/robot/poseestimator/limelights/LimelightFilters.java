package frc.robot.poseestimator.limelights;

//import edu.wpi.first.math.geometry.Pose3d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.Field;

public class LimelightFilters {

	protected static boolean isLimelightOutputInTolerance(LimelightRawData limelightRawData) {
		// ! THIS SHOULDN'T BE COMMENTED OUT
		// ! this is a placeholder since this filter is depended on the poseestimatorx
		return true;
//		Pose3d currentPoseObservation = NetworkTables...;
//
//		Pose3d limelightPosition = limelightRawData.estimatedPose();
//		Transform3d transformDifference = limelightPosition.minus(currentPoseObservation);
//		Rotation2d rotationDifference = Rotation2d.fromRadians(limelightPosition.getRotation().getZ() - currentPoseObservation.getRotation().getZ());
//
//		return transformDifference.getTranslation().getNorm() <= VisionConstants.positionNormTolerance
//			&& rotationDifference.getDegrees() <= VisionConstants.rotationTolerance.getDegrees();
	}

	protected static boolean isPitchZero(LimelightRawData limelightRawData) {
		return Math.abs(limelightRawData.estimatedPose().getRotation().getY()) <= VisionConstants.PITCH_TOLERANCE.getRadians();
	}

	protected static boolean isRollZero(LimelightRawData limelightRawData) {
		return Math.abs(limelightRawData.estimatedPose().getRotation().getX()) <= VisionConstants.ROLL_TOLERANCE.getRadians();
	}

	protected static boolean isAprilTagInProperHeight(LimelightRawData limelightRawData) {
		double aprilTagHeightConfidence = Math.abs(limelightRawData.aprilTagHeight() - Field.APRIL_TAG_HEIGHT_METERS);
		return aprilTagHeightConfidence <= VisionConstants.APRIL_TAG_HEIGHT_TOLERANCE_METERS;
	}

	protected static boolean isRobotOnGround(LimelightRawData limelightRawData) {
		return limelightRawData.estimatedPose().getY() <= VisionConstants.ROBOT_TO_GROUND_TOLERANCE;
	}

}
