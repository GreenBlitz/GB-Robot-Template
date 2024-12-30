package frc.robot.subsystems.swerve.factories.swerveconstants;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.SwerveConstants;

public class RealSwerveConstants {

	public static final double VELOCITY_AT_12_VOLTS_METERS_PER_SECOND = 5.054;
	private static final Rotation2d MAX_ROTATION_VELOCITY = Rotation2d.fromRadians(10);

	private static final PIDConstants TRANSLATION_METERS_PID_CONSTANTS = new PIDConstants(6, 0, 0);
	private static final PIDConstants ROTATION_DEGREES_PID_CONSTANTS = new PIDConstants(6, 0, 0);

	protected static SwerveConstants getSwerveConstants(String logPath) {
		return new SwerveConstants(
			logPath,
			VELOCITY_AT_12_VOLTS_METERS_PER_SECOND,
			MAX_ROTATION_VELOCITY,
			TRANSLATION_METERS_PID_CONSTANTS,
			ROTATION_DEGREES_PID_CONSTANTS
		);
	}

}
