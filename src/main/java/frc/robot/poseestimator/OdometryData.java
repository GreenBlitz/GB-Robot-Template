package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;

public class OdometryData {

	private double timestampSeconds = 0;
	private SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
	private Optional<Rotation2d> imuYaw = Optional.empty();

	public OdometryData() {}

	public OdometryData(double timestampSeconds, SwerveModulePosition[] wheelPositions, Optional<Rotation2d> imuYaw) {
		this.timestampSeconds = timestampSeconds;
		this.wheelPositions = wheelPositions;
		this.imuYaw = imuYaw;
	}

	public double getTimestampSeconds() {
		return timestampSeconds;
	}

	public SwerveModulePosition[] getWheelPositions() {
		return wheelPositions;
	}

	public Optional<Rotation2d> getIMUYaw() {
		return imuYaw;
	}

	public void setTimestamp(double timestampSeconds) {
		this.timestampSeconds = timestampSeconds;
	}

	public void setWheelPositions(SwerveModulePosition[] wheelPositions) {
		this.wheelPositions = wheelPositions;
	}

	public void setIMUYaw(Optional<Rotation2d> imuYaw) {
		this.imuYaw = imuYaw;
	}

	public void setIMUYaw(Rotation2d imuYaw) {
		setIMUYaw(Optional.of(imuYaw));
	}

}
