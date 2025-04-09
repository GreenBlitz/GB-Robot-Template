package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;

public class OdometryData {

	private SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
	private Optional<Rotation2d> gyroAngle = Optional.empty();
	private double timestamp = 0;

	public OdometryData() {}

	public OdometryData(SwerveModulePosition[] swerveModulePositions, Optional<Rotation2d> gyroAngle, double timestamp) {
		this.wheelPositions = swerveModulePositions;
		this.gyroAngle = gyroAngle;
		this.timestamp = timestamp;
	}

	public SwerveModulePosition[] getWheelPositions() {
		return wheelPositions;
	}

	public double getTimestamp() {
		return timestamp;
	}

	public Optional<Rotation2d> getGyroAngle() {
		return gyroAngle;
	}

	public void setWheelPositions(SwerveModulePosition[] wheelPositions) {
		this.wheelPositions = wheelPositions;
	}

	public void setGyroAngle(Optional<Rotation2d> gyroAngle) {
		this.gyroAngle = gyroAngle;
	}

	public void setTimestamp(double timestamp) {
		this.timestamp = timestamp;
	}

}
