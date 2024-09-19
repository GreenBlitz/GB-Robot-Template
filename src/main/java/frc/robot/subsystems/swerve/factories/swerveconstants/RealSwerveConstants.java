package frc.robot.subsystems.swerve.factories.swerveconstants;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveName;

public class RealSwerveConstants {

	public static final double VELOCITY_AT_12_VOLTS_METERS_PER_SECOND = 5.054;
	private static final Rotation2d MAX_ROTATION_VELOCITY = Rotation2d.fromRadians(10);

	private static final PIDConstants TRANSLATION_METERS_PID_CONSTANTS = new PIDConstants(6, 0, 0);
	private static final PIDConstants ROTATION_DEGREES_PID_CONSTANTS = new PIDConstants(6, 0, 0);
	private static final Pair<Translation2d, Double>[] DISCRITIZATION_POINTS = new Pair[] {
		new Pair<>(new Translation2d(0, 0), 1.0),
		new Pair<>(new Translation2d(0, 15), 3.2),
		new Pair<>(new Translation2d(6, 0), 2.5),
		new Pair<>(new Translation2d(5, 15), 3.2),
		new Pair<>(new Translation2d(6.5, 15), 4.0),
		new Pair<>(new Translation2d(6.5, 11), 4.0)};

	protected static SwerveConstants getSwerveConstants(SwerveName swerveName) {
		return new SwerveConstants(
			swerveName,
			VELOCITY_AT_12_VOLTS_METERS_PER_SECOND,
			MAX_ROTATION_VELOCITY,
			TRANSLATION_METERS_PID_CONSTANTS,
			ROTATION_DEGREES_PID_CONSTANTS,
			DISCRITIZATION_POINTS
		);
	}

}
