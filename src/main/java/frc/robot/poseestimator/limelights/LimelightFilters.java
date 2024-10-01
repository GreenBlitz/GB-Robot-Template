package frc.robot.poseestimator.limelights;

//import edu.wpi.first.math.geometry.Pose3d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.constants.Field;

public class LimelightFilters {

	protected static boolean isLimelightOutputInTolerance(LimelightRawData limelightRawData, Pose2d estimatedPose) {
		// ! THIS SHOULDN'T BE COMMENTED OUT
		// ! this is a placeholder since this filter is depended on the poseestimator

		Pose2d limelightPosition = limelightRawData.estimatedPose().toPose2d();
		Transform2d transformDifference = limelightPosition.minus(estimatedPose);
		Rotation2d rotationDifference = limelightPosition.getRotation().minus(estimatedPose.getRotation());
		return transformDifference.getTranslation().getNorm() <= VisionConstants.POSITION_NORM_TOLERANCE
			&& rotationDifference.getDegrees() <= VisionConstants.ROTATION_TOLERANCE.getDegrees();
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
		return limelightRawData.estimatedPose().getZ() <= VisionConstants.ROBOT_TO_GROUND_TOLERANCE;
	}

}
