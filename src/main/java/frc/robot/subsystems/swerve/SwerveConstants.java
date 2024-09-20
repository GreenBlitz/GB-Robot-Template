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
	Pair<Translation2d, Double>[] discretizationPointsArray,
	Translation2d[] modulesLocations,
	double driveRadiusMeters,
	SwerveDriveKinematics kinematics,
	HolonomicPathFollowerConfig holonomicPathFollowerConfig
) {
	public SwerveConstants(
		SwerveName swerveName,
		double velocityAt12VoltsMetersPerSecond,
		Rotation2d maxRotationalVelocityPerSecond,
		PIDConstants translationMetersPIDConstants,
		PIDConstants rotationDegreesPIDConstants,
		Pair<Translation2d, Double>[] discretizationPointsArray,
		Translation2d[] modulesLocations,
		double driveRadiusMeters
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
			discretizationPointsArray,
			modulesLocations,
			driveRadiusMeters,
			new SwerveDriveKinematics(modulesLocations),
			new HolonomicPathFollowerConfig(
				translationMetersPIDConstants,
				rotationDegreesPIDConstants,
				velocityAt12VoltsMetersPerSecond,
				driveRadiusMeters,
				REPLANNING_CONFIG
			)
		);

		this.rotationDegreesPIDController.enableContinuousInput(-MathConstants.HALF_CIRCLE.getDegrees(), MathConstants.HALF_CIRCLE.getDegrees());
	}

	public static final double AIM_ASSIST_MAGNITUDE_FACTOR = 4;
	static final Rotation2d WHEEL_RADIUS_CALIBRATION_VELOCITY_PER_SECOND = Rotation2d.fromRotations(0.5);
	static final double DRIVE_NEUTRAL_DEADBAND = 0.05;
	static final Rotation2d ROTATION_NEUTRAL_DEADBAND = Rotation2d.fromRadians(0.05);
	static final PathConstraints REAL_TIME_CONSTRAINTS = new PathConstraints(2.5, 2.5, 4, 4);
	static final double CLOSE_TO_TARGET_POSITION_DEADBAND_METERS = 0.5;
	private static final ReplanningConfig REPLANNING_CONFIG = new ReplanningConfig(true, true);


}
