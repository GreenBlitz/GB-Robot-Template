package frc.robot.subsystems.swerve.states;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.subsystems.swerve.factories.constants.RealSwerveConstants;

public enum DriveAcceleration {

	DEFAULT(RealSwerveConstants.ACCELERATION_AT_12_VOLTS_METERS_PER_SECOND_SQUARED),
	SKID_PREVENTION(RealSwerveConstants.MAX_MODULE_SKID_ACCELERATION_METERS_PER_SECOND_SQUARED);

	private final double maxAccelerationMetersPerSecondSquared;
	private final SlewRateLimiter slewRateLimiter;

	DriveAcceleration(double maxAccelerationMetersPerSecondSquared) {
		this.maxAccelerationMetersPerSecondSquared = maxAccelerationMetersPerSecondSquared;
		slewRateLimiter = new SlewRateLimiter(maxAccelerationMetersPerSecondSquared);
	}

	public double getMaxAccelerationMetersPerSecondSquared() {
		return maxAccelerationMetersPerSecondSquared;
	}

	public SlewRateLimiter getSlewRateLimiter() {
		return slewRateLimiter;
	}

}
