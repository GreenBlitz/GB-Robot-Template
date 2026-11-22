package frc.robot.subsystems.swerve.states;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.subsystems.swerve.factories.constants.RealSwerveConstants;

public enum DriveAcceleration {

	DEFAULT(
		RealSwerveConstants.ACCELERATION_AT_12_VOLTS_METERS_PER_SECOND_SQUARED,
		RealSwerveConstants.ACCELERATION_AT_12_VOLTS_METERS_PER_SECOND_SQUARED,
		RealSwerveConstants.ACCELERATION_AT_12_VOLTS_METERS_PER_SECOND_SQUARED,
		RealSwerveConstants.ACCELERATION_AT_12_VOLTS_METERS_PER_SECOND_SQUARED
	),
	SKID_PREVENTION(
		RealSwerveConstants.FRONT_LEFT_MODULE_ACCELERATION_LIMIT_METERS_PER_SECOND_SQUARED,
		RealSwerveConstants.FRONT_RIGHT_MODULE_ACCELERATION_LIMIT_METERS_PER_SECOND_SQUARED,
		RealSwerveConstants.BACK_LEFT_MODULE_ACCELERATION_LIMIT_METERS_PER_SECOND_SQUARED,
		RealSwerveConstants.BACK_RIGHT_MODULE_ACCELERATION_LIMIT_METERS_PER_SECOND_SQUARED
	);

	private final double[] maxAccelerationsMetersPerSecondSquared;
	private final SlewRateLimiter[] slewRateLimiters = new SlewRateLimiter[4];

	DriveAcceleration(double frontLeft, double frontRight, double backLeft, double backRight) {
		this.maxAccelerationsMetersPerSecondSquared = new double[] {frontLeft, frontRight, backLeft, backRight};
	}

	public SlewRateLimiter[] createLimiters() {
		SlewRateLimiter[] limiters = new SlewRateLimiter[maxAccelerationsMetersPerSecondSquared.length];
		for (int i = 0; i < limiters.length; i++) {
			limiters[i] = new SlewRateLimiter(maxAccelerationsMetersPerSecondSquared[i], -maxAccelerationsMetersPerSecondSquared[i], 0);
		}
		return limiters;
	}

	public SlewRateLimiter[] getSlewRateLimiters() {
		return slewRateLimiters;
	}

	public SlewRateLimiter getSlewRateLimiter(int moduleIndex) {
		return slewRateLimiters[moduleIndex];
	}

	public double getMaxAccelerationMetersPerSecondSquared(int moduleIndex) {
		return maxAccelerationsMetersPerSecondSquared[moduleIndex];
	}

}
