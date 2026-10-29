package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;

import java.util.function.Supplier;

public enum HeadingSource {

	IMU,
	LIMELIGHT_IMU,
	VISION,
	SWERVE;

	public static Supplier<Rotation2d> handleHeadingSource(HeadingSource headingSource, Robot robot) {
		return switch (headingSource) {
			case IMU -> () -> robot.getImuWrapper().getAbsoluteYaw().getValue();
			case LIMELIGHT_IMU -> () -> new Rotation2d();
			case SWERVE -> () -> robot.getPoseEstimator().getOdometryPose().getRotation();
			case VISION -> () -> robot.getPoseEstimator().getLastVisionObservation().robotPose().getRotation();
		};
	}

}
