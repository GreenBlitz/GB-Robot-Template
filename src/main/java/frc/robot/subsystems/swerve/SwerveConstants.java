package frc.robot.subsystems.swerve;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.constants.MathConstants;
import frc.robot.subsystems.swerve.gyro.GyroConstants;

public record SwerveConstants(
	String logPath,
	String stateLogPath,
	String velocityLogPath,
	String gyroLogPath,
	double velocityAt12VoltsMetersPerSecond,
	Rotation2d maxRotationalVelocityPerSecond,
	PIDController xMetersPIDController,
	PIDController yMetersPIDController,
	PIDController rotationDegreesPIDController,
	HolonomicPathFollowerConfig holonomicPathFollowerConfig,
	Pair<Translation2d, Double>[] discritizationPointsArray
) {

	public SwerveConstants(
		SwerveName swerveName,
		double velocityAt12VoltsMetersPerSecond,
		Rotation2d maxRotationalVelocityPerSecond,
		PIDConstants translationMetersPIDConstants,
		PIDConstants rotationDegreesPIDConstants,
		Pair<Translation2d, Double>[] discritizationPointsArray
	) {
		this(
			swerveName.getLogPath(),
			swerveName.getLogPath() + "State/",
			swerveName.getLogPath() + "Velocity/",
			swerveName.getLogPath() + GyroConstants.LOG_PATH_ADDITION,
			velocityAt12VoltsMetersPerSecond,
			maxRotationalVelocityPerSecond,
			new PIDController(translationMetersPIDConstants.kP, translationMetersPIDConstants.kI, translationMetersPIDConstants.kD),
			new PIDController(translationMetersPIDConstants.kP, translationMetersPIDConstants.kI, translationMetersPIDConstants.kD),
			new PIDController(rotationDegreesPIDConstants.kP, rotationDegreesPIDConstants.kI, rotationDegreesPIDConstants.kD),
			new HolonomicPathFollowerConfig(
				translationMetersPIDConstants,
				rotationDegreesPIDConstants,
				velocityAt12VoltsMetersPerSecond,
				DRIVE_RADIUS_METERS,
				REPLANNING_CONFIG
			),
			discritizationPointsArray
		);

		this.rotationDegreesPIDController.enableContinuousInput(-MathConstants.HALF_CIRCLE.getDegrees(), MathConstants.HALF_CIRCLE.getDegrees());
	}

	static final Rotation2d WHEEL_RADIUS_CALIBRATION_VELOCITY = Rotation2d.fromRotations(0.5);

	public static final double AIM_ASSIST_MAGNITUDE_FACTOR = 4;

	static final double DRIVE_NEUTRAL_DEADBAND = 0.05;
	static final Rotation2d ROTATION_NEUTRAL_DEADBAND = Rotation2d.fromRadians(0.05);

	private static final double MODULE_X_DISTANCE_FROM_CENTER = 0.27833;
	private static final double MODULE_Y_DISTANCE_FROM_CENTER = 0.34733;
	static final double DRIVE_RADIUS_METERS = Math.hypot(MODULE_X_DISTANCE_FROM_CENTER, MODULE_Y_DISTANCE_FROM_CENTER);
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
	public static final Translation2d[] LOCATIONS = {
		FRONT_LEFT_TRANSLATION2D,
		FRONT_RIGHT_TRANSLATION2D,
		BACK_LEFT_TRANSLATION2D,
		BACK_RIGHT_TRANSLATION2D};
	public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(LOCATIONS);

	private static final ReplanningConfig REPLANNING_CONFIG = new ReplanningConfig(true, true);
	static final PathConstraints REAL_TIME_CONSTRAINTS = new PathConstraints(2.5, 2.5, 4, 4);
	static final double CLOSE_TO_TARGET_POSITION_DEADBAND_METERS = 0.5;

}
