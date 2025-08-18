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
	private Matrix<N3, N1> stdDevs;

	public RobotPoseObservation(double timestampSeconds, Pose2d robotPose, Matrix<N3, N1> standardDeviations) {
		setObservationValues(timestampSeconds, robotPose, standardDeviations);
	}

	public RobotPoseObservation() {
		this(0, new Pose2d(), MatBuilder.fill(Nat.N3(), Nat.N1(), 0, 0, 0));
	}

	public void setObservationValues(double timestampSeconds, Pose2d robotPose, Matrix<N3, N1> stdDevs) {
		this.timestampSeconds = timestampSeconds;
		this.robotPose = robotPose;
		this.stdDevs = stdDevs;
	}

	public double getTimestampSeconds() {
		return timestampSeconds;
	}

	public Pose2d getRobotPose() {
		return robotPose;
	}

	public Matrix<N3, N1> getStdDevs() {
		return stdDevs;
	}

}
