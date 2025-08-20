package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.utils.math.StandardDeviations2D;

public record RobotPoseObservation(double timestampSeconds, Pose2d robotPose, StandardDeviations2D stdDevs) {

	public RobotPoseObservation() {
		this(0, new Pose2d(), new StandardDeviations2D());
	}

}
