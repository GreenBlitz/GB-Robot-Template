package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;

public class OdometryData {

	private double timestamp = 0;
	private SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
	private Optional<Rotation2d> imuYaw = Optional.empty();

	public OdometryData() {}

	public OdometryData(double timestamp, SwerveModulePosition[] wheelPositions, Optional<Rotation2d> imuYaw) {
		this.timestamp = timestamp;
		this.wheelPositions = wheelPositions;
		this.imuYaw = imuYaw;
	}

	public double getTimestamp() {
		return timestamp;
	}

	public SwerveModulePosition[] getWheelPositions() {
		return wheelPositions;
	}

	public Optional<Rotation2d> getIMUYaw() {
		return imuYaw;
	}

	public void setTimestamp(double timestamp) {
		this.timestamp = timestamp;
	}

	public void setWheelPositions(SwerveModulePosition[] wheelPositions) {
		this.wheelPositions = wheelPositions;
	}

	public void setIMUYaw(Optional<Rotation2d> imuYaw) {
		this.imuYaw = imuYaw;
	}

	public void setIMUYaw(Rotation2d IMUYaw) {
		setIMUYaw(Optional.of(IMUYaw));
	}

}
