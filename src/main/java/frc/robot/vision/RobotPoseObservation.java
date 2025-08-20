package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;

public record RobotPoseObservation(double timestampSeconds, Pose2d robotPose, StdDevs stdDevs) {

	public RobotPoseObservation() {
		this(0, new Pose2d(), new StdDevs(0.0, 0.0, 0.0));
	}

}
