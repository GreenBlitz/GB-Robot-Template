package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;

public class OdometryData {

	private double timestamp = 0;
	private SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
	private Optional<Rotation2d> IMUYaw = Optional.empty();

	public OdometryData() {}

	public OdometryData(double timestamp, SwerveModulePosition[] swerveModulePositions, Optional<Rotation2d> IMUYaw) {
		this.timestamp = timestamp;
		this.wheelPositions = swerveModulePositions;
		this.IMUYaw = IMUYaw;
	}

	public double getTimestamp() {
		return timestamp;
	}

	public SwerveModulePosition[] getWheelPositions() {
		return wheelPositions;
	}

	public Optional<Rotation2d> getIMUYaw() {
		return IMUYaw;
	}

	public void setTimestamp(double timestamp) {
		this.timestamp = timestamp;
	}

	public void setWheelPositions(SwerveModulePosition[] wheelPositions) {
		this.wheelPositions = wheelPositions;
	}

	public void setIMUYaw(Optional<Rotation2d> IMUYaw) {
		this.IMUYaw = IMUYaw;
	}

	public void setIMUYaw(Rotation2d IMUYaw) {
		setIMUYaw(Optional.of(IMUYaw));
	}

}
