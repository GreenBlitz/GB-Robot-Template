package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;

public class OdometryData {

	private SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
	private Optional<Rotation2d> gyroYaw = Optional.empty();
	private double timestamp = 0;

	public OdometryData() {}

	public OdometryData(SwerveModulePosition[] swerveModulePositions, Optional<Rotation2d> gyroYaw, double timestamp) {
		this.wheelPositions = swerveModulePositions;
		this.gyroYaw = gyroYaw;
		this.timestamp = timestamp;
	}

	public SwerveModulePosition[] getWheelPositions() {
		return wheelPositions;
	}

	public double getTimestamp() {
		return timestamp;
	}

	public Optional<Rotation2d> getGyroYaw() {
		return gyroYaw;
	}

	public void setWheelPositions(SwerveModulePosition[] wheelPositions) {
		this.wheelPositions = wheelPositions;
	}

	public void setGyroYaw(Optional<Rotation2d> gyroYaw) {
		this.gyroYaw = gyroYaw;
	}

	public void setTimestamp(double timestamp) {
		this.timestamp = timestamp;
	}

}
