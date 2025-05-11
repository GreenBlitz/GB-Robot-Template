package frc.robot.subsystems.swerve.factories.constants;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.SwerveConstants;

public class RealSwerveConstants {

	public static final double VELOCITY_AT_12_VOLTS_METERS_PER_SECOND = 3.7;
	public static final double ACCELERATION_AT_12_VOLTS_METERS_PER_SECOND_SQUARED = 1.9;
	public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND = 4;
	public static final Rotation2d MAX_ROTATIONAL_VELOCITY_PER_SECOND = Rotation2d.fromRadians(7.9);

	private static final PIDConstants TRANSLATION_METERS_PID_CONSTANTS = new PIDConstants(2, 7.9, 0, 0.03);
	private static final PIDConstants ROTATION_DEGREES_PID_CONSTANTS = new PIDConstants(2.7, 0.5, 0, 2);

	static SwerveConstants getSwerveConstants(String logPath) {
		return new SwerveConstants(
			logPath,
			VELOCITY_AT_12_VOLTS_METERS_PER_SECOND,
			MAX_ROTATIONAL_VELOCITY_PER_SECOND,
			TRANSLATION_METERS_PID_CONSTANTS,
			ROTATION_DEGREES_PID_CONSTANTS
		);
	}

}
