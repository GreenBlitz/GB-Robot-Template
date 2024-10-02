package frc.robot.vision.limelights;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.Field;

public class LimelightFilters {

	protected static boolean isLimelightOutputInTolerance(LimelightRawData limelightRawData, Pose2d estimatedPose) {
		Pose3d limelightPosition = limelightRawData.estimatedPose();
		Pose3d estimatedPose3d = new Pose3d(
			estimatedPose.getX(),
			estimatedPose.getY(),
			0,
			new Rotation3d(0, 0, estimatedPose.getRotation().getRadians())
		);
		Transform3d transformDifference = limelightPosition.minus(estimatedPose3d);
		Rotation3d rotationDifference = limelightPosition.getRotation().minus(estimatedPose3d.getRotation());
		return transformDifference.getTranslation().getNorm() <= LimeLightConstants.POSITION_NORM_TOLERANCE
			&& getRotationNorm(rotationDifference) <= LimeLightConstants.ROTATION_NORM_TOLERANCE;
	}

	private static double getRotationNorm(Rotation3d angle) {
		return Math.pow(Math.pow(angle.getX(), 2) + Math.pow(angle.getY(), 2) + Math.pow(angle.getZ(), 2), 0.5);
	}

	protected static boolean isPitchZero(LimelightRawData limelightRawData) {
		return Math.abs(limelightRawData.estimatedPose().getRotation().getY()) <= LimeLightConstants.PITCH_TOLERANCE.getRadians();
	}

	protected static boolean isRollZero(LimelightRawData limelightRawData) {
		return Math.abs(limelightRawData.estimatedPose().getRotation().getX()) <= LimeLightConstants.ROLL_TOLERANCE.getRadians();
	}

	protected static boolean isAprilTagInProperHeight(LimelightRawData limelightRawData) {
		double aprilTagHeightConfidence = Math.abs(limelightRawData.aprilTagHeight() - Field.APRIL_TAG_HEIGHT_METERS);
		return aprilTagHeightConfidence <= LimeLightConstants.APRIL_TAG_HEIGHT_TOLERANCE_METERS;
	}

	protected static boolean isRobotOnGround(LimelightRawData limelightRawData) {
		return limelightRawData.estimatedPose().getZ() <= LimeLightConstants.ROBOT_TO_GROUND_TOLERANCE;
	}

}
