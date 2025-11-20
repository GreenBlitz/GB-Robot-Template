package frc.robot.subsystems.swerve.factories.constants;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.SwerveConstants;

public class SimulationSwerveConstants {

	public static final double VELOCITY_AT_12_VOLTS_METERS_PER_SECOND = 4.133;
	public static final Rotation2d MAX_ROTATIONAL_VELOCITY_PER_SECOND = Rotation2d.fromRadians(8.742);

	private static final PIDConstants TRANSLATION_METERS_PID_CONSTANTS = new PIDConstants(5.5, 0, 0);
	private static final PIDConstants ROTATION_DEGREES_PID_CONSTANTS = new PIDConstants(5.5, 0, 0);

	public static SwerveConstants getSwerveConstants(String logPath) {
		return new SwerveConstants(
			logPath,
			VELOCITY_AT_12_VOLTS_METERS_PER_SECOND,
			MAX_ROTATIONAL_VELOCITY_PER_SECOND,
			TRANSLATION_METERS_PID_CONSTANTS,
			ROTATION_DEGREES_PID_CONSTANTS
		);
	}

}
