package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;

public class OdometryData {

	public SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
	public Optional<Rotation2d> gyroAngle = Optional.empty();
	public double timestamp = 0;

	public OdometryData() {}

	public OdometryData(SwerveModulePosition[] swerveModulePositions, Optional<Rotation2d> gyroAngle, double timestamp) {
		this.wheelPositions = swerveModulePositions;
		this.gyroAngle = gyroAngle;
		this.timestamp = timestamp;
	}

}
