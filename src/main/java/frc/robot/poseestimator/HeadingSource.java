package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;

import java.util.function.Supplier;

public enum HeadingSource {

	IMU,
	VISION,
	SWERVE;

	public static Supplier<Rotation2d> handleHeadingSource(HeadingSource headingSource, Robot robot) {
		return switch (headingSource) {
			case IMU -> () -> robot.getIMU().getWrappedYaw().getValue();
			case SWERVE -> () -> robot.getPoseEstimator().getSwerveYaw();
			case VISION -> () -> robot.getPoseEstimator().getLastVisionObservation().robotPose().getRotation();
		};
	}

}
