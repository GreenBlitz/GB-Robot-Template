package frc.robot.newvision;

import edu.wpi.first.math.geometry.Pose2d;

public class RobotPoseObservation {

	private double timestampSeconds;
	private Pose2d robotPose;
	private Pose2d standardDeviations;

	public RobotPoseObservation(double timestampSeconds, Pose2d robotPose, Pose2d standardDeviations) {
		setObservationValues(timestampSeconds, robotPose, standardDeviations);
	}

	public RobotPoseObservation() {
		this(0, new Pose2d(), new Pose2d());
	}

	public void setObservationValues(double timestampSeconds, Pose2d robotPose, Pose2d standardDeviations) {
		this.timestampSeconds = timestampSeconds;
		this.robotPose = robotPose;
		this.standardDeviations = standardDeviations;
	}

	public double getTimestampSeconds() {
		return timestampSeconds;
	}

	public Pose2d getRobotPose() {
		return robotPose;
	}

	public Pose2d getStandardDeviations() {
		return standardDeviations;
	}

}
