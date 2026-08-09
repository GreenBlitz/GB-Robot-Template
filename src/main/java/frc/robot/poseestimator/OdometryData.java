package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.util.Optional;

public class OdometryData {

	private double timestampSeconds = 0;
	private SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
	private SwerveModuleState[] wheelStates = new SwerveModuleState[4];
	private Optional<Rotation3d> imuOrientation = Optional.empty();
	private Optional<Translation3d> imu3DAccelerationG = Optional.empty();

	public OdometryData() {}

	public OdometryData(
		double timestampSeconds,
		SwerveModulePosition[] wheelPositions,
		SwerveModuleState[] wheelStates,
		Optional<Rotation3d> imuOrientation,
		Optional<Translation3d> imu3DAccelerationG
	) {
		this.timestampSeconds = timestampSeconds;
		this.wheelPositions = wheelPositions;
		this.wheelStates = wheelStates;
		this.imuOrientation = imuOrientation;
		this.imu3DAccelerationG = imu3DAccelerationG;
	}

	public double getTimestampSeconds() {
		return timestampSeconds;
	}

	public SwerveModulePosition[] getWheelPositions() {
		return wheelPositions;
	}

	public SwerveModuleState[] getWheelStates() {
		return wheelStates;
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

	public void setWheelStates(SwerveModuleState[] wheelStates) {
		this.wheelStates = wheelStates;
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
