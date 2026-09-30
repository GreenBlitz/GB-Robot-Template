package frc.robot.poseestimator;

import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

import java.util.Optional;

public class OdometryData {

	private double timestampSeconds = 0;
	private SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
	private SwerveModuleVelocity[] wheelVelocities = new SwerveModuleVelocity[4];
	private Optional<Rotation3d> imuOrientation = Optional.empty();
	private Optional<Translation3d> imu3DAccelerationG = Optional.empty();

	public OdometryData() {}

	public OdometryData(
		double timestampSeconds,
		SwerveModulePosition[] wheelPositions,
		SwerveModuleVelocity[] wheelVelocities,
		Optional<Rotation3d> imuOrientation,
		Optional<Translation3d> imu3DAccelerationG
	) {
		this.timestampSeconds = timestampSeconds;
		this.wheelPositions = wheelPositions;
		this.wheelVelocities = wheelVelocities;
		this.imuOrientation = imuOrientation;
		this.imu3DAccelerationG = imu3DAccelerationG;
	}

	public double getTimestampSeconds() {
		return timestampSeconds;
	}

	public SwerveModulePosition[] getWheelPositions() {
		return wheelPositions;
	}

	public SwerveModuleVelocity[] getWheelVelocities() {
		return wheelVelocities;
	}

	public Optional<Rotation3d> getIMUOrientation() {
		return imuOrientation;
	}

	public Optional<Translation3d> getIMU3DAccelerationG() {
		return imu3DAccelerationG;
	}

	public void setTimestamp(double timestampSeconds) {
		this.timestampSeconds = timestampSeconds;
	}

	public void setWheelPositions(SwerveModulePosition[] wheelPositions) {
		this.wheelPositions = wheelPositions;
	}

	public void setWheelVelocities(SwerveModuleVelocity[] wheelVelocities) {
		this.wheelVelocities = wheelVelocities;
	}

	public void setIMUOrientation(Optional<Rotation3d> imuOrientation) {
		this.imuOrientation = imuOrientation;
	}

	public void setIMUOrientation(Rotation3d imuOrientation) {
		setIMUOrientation(Optional.of(imuOrientation));
	}

	public void setIMU3DAcceleration(Optional<Translation3d> imu3DAccelerationG) {
		this.imu3DAccelerationG = imu3DAccelerationG;
	}

	public void setIMU3DAcceleration(Translation3d imu3DAccelerationG) {
		setIMU3DAcceleration(Optional.of(imu3DAccelerationG));
	}

}
