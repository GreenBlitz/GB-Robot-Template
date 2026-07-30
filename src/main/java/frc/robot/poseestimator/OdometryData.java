package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.util.Optional;

public class OdometryData {

	private double timestampSeconds = 0;
	private SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
	private SwerveModuleState[] wheelStates = new SwerveModuleState[4];
	private Optional<Rotation3d> imuOrientation = Optional.empty();
	private Optional<Translation2d> imuXYAccelerationG = Optional.empty();

	public OdometryData() {}

	public OdometryData(
		double timestampSeconds,
		SwerveModulePosition[] wheelPositions,
		SwerveModuleState[] wheelStates,
		Optional<Rotation3d> imuOrientation,
		Optional<Translation2d> imuXYAccelerationG
	) {
		this.timestampSeconds = timestampSeconds;
		this.wheelPositions = wheelPositions;
		this.wheelStates = wheelStates;
		this.imuOrientation = imuOrientation;
		this.imuXYAccelerationG = imuXYAccelerationG;
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

	public Optional<Translation2d> getIMUXYAccelerationG() {
		return imuXYAccelerationG;
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

	public void setIMUXYAcceleration(Optional<Translation2d> imuXYAccelerationG) {
		this.imuXYAccelerationG = imuXYAccelerationG;
	}

	public void setIMUXYAcceleration(Translation2d imuXYAccelerationG) {
		setIMUXYAcceleration(Optional.of(imuXYAccelerationG));
	}

}
