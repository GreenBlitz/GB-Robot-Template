package frc.robot.newvision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class RobotPoseObservation {

	private double timestampSeconds;
	private Pose2d robotPose;
	private Matrix<N1, N3> standardDeviations;

	public RobotPoseObservation(double timestampSeconds, Pose2d robotPose, Matrix<N1, N3> standardDeviations) {
		setObservationValues(timestampSeconds, robotPose, standardDeviations);
	}

	public RobotPoseObservation() {
		this(0, new Pose2d(), MatBuilder.fill(Nat.N1(), Nat.N3(), 0, 0, 0));
	}

	public void setObservationValues(double timestampSeconds, Pose2d robotPose, Matrix<N1, N3> standardDeviations) {
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

	public Matrix<N1, N3> getStandardDeviations() {
		return standardDeviations;
	}

}
