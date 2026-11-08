package frc.robot.subsystems.swerve.states;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.subsystems.swerve.factories.constants.RealSwerveConstants;

public enum DriveAcceleration {

	NORMAL(RealSwerveConstants.ACCELERATION_AT_12_VOLTS_METERS_PER_SECOND_SQUARED),
	SKID_PREVENTION(RealSwerveConstants.MAX_SKID_ACCELERATION_METERS_PER_SECOND_SQUARED);

	private final double maxAcceleration;
	private final SlewRateLimiter slewRateLimiter;

	DriveAcceleration(double maxAcceleration) {
		this.maxAcceleration = maxAcceleration;
		slewRateLimiter = new SlewRateLimiter(maxAcceleration);
	}

	public double getMaxAcceleration() {
		return maxAcceleration;
	}

	public SlewRateLimiter getSlewRateLimiter() {
		return slewRateLimiter;
	}

}
