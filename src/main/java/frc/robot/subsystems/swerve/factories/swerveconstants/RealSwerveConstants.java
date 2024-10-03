package frc.robot.subsystems.swerve.factories.swerveconstants;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveType;

public class RealSwerveConstants {

	public static final double VELOCITY_AT_12_VOLTS_METERS_PER_SECOND = 5.054;
	private static final Rotation2d MAX_ROTATION_VELOCITY = Rotation2d.fromRadians(10);

	private static final PIDConstants TRANSLATION_METERS_PID_CONSTANTS = new PIDConstants(6, 0, 0);
	private static final PIDConstants ROTATION_DEGREES_PID_CONSTANTS = new PIDConstants(6, 0, 0);

	private static final double MODULE_X_DISTANCE_FROM_CENTER = 0.27833;
	private static final double MODULE_Y_DISTANCE_FROM_CENTER = 0.34733;
	private static final double DRIVE_RADIUS_METERS = Math.hypot(MODULE_X_DISTANCE_FROM_CENTER, MODULE_Y_DISTANCE_FROM_CENTER);
	private static final Translation2d FRONT_LEFT_TRANSLATION2D = new Translation2d(
		MODULE_X_DISTANCE_FROM_CENTER,
		MODULE_Y_DISTANCE_FROM_CENTER
	);
	private static final Translation2d FRONT_RIGHT_TRANSLATION2D = new Translation2d(
		MODULE_X_DISTANCE_FROM_CENTER,
		-MODULE_Y_DISTANCE_FROM_CENTER
	);
	private static final Translation2d BACK_LEFT_TRANSLATION2D = new Translation2d(
		-MODULE_X_DISTANCE_FROM_CENTER,
		MODULE_Y_DISTANCE_FROM_CENTER
	);
	private static final Translation2d BACK_RIGHT_TRANSLATION2D = new Translation2d(
		-MODULE_X_DISTANCE_FROM_CENTER,
		-MODULE_Y_DISTANCE_FROM_CENTER
	);
	private static final Translation2d[] LOCATIONS = {
		FRONT_LEFT_TRANSLATION2D,
		FRONT_RIGHT_TRANSLATION2D,
		BACK_LEFT_TRANSLATION2D,
		BACK_RIGHT_TRANSLATION2D};

	protected static SwerveConstants getSwerveConstants(SwerveType swerveType) {
		return new SwerveConstants(
			swerveType,
			VELOCITY_AT_12_VOLTS_METERS_PER_SECOND,
			MAX_ROTATION_VELOCITY,
			TRANSLATION_METERS_PID_CONSTANTS,
			ROTATION_DEGREES_PID_CONSTANTS,
			LOCATIONS,
			DRIVE_RADIUS_METERS
		);
	}

}
