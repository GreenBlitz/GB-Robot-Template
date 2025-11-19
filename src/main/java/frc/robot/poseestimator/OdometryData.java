package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;

public class OdometryData {

	private SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
	private Optional<Rotation2d> IMUYaw = Optional.empty();
	private double timestamp = 0;

	public OdometryData() {}

	public OdometryData(SwerveModulePosition[] swerveModulePositions, Optional<Rotation2d> IMUYaw, double timestamp) {
		this.wheelPositions = swerveModulePositions;
		this.IMUYaw = IMUYaw;
		this.timestamp = timestamp;
	}

	public SwerveModulePosition[] getWheelPositions() {
		return wheelPositions;
	}

	public double getTimestamp() {
		return timestamp;
	}

	public Optional<Rotation2d> getIMUYaw() {
		return IMUYaw;
	}

	public void setWheelPositions(SwerveModulePosition[] wheelPositions) {
		this.wheelPositions = wheelPositions;
	}

	public void setTimestamp(double timestamp) {
		this.timestamp = timestamp;
	}

	public void setIMUYaw(Optional<Rotation2d> IMUYaw) {
		this.IMUYaw = IMUYaw;
	}

	public void setIMUYaw(Rotation2d IMUYaw) {
		setIMUYaw(Optional.of(IMUYaw));
	}

}
