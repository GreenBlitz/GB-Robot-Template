package frc.robot.subsystems.swerve;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.MathConstants;

public record SwerveConstants(
	String logPath,
	String stateLogPath,
	String velocityLogPath,
	double velocityAt12VoltsMetersPerSecond,
	Rotation2d maxRotationalVelocityPerSecond,
	PIDController xMetersPIDController,
	PIDController yMetersPIDController,
	PIDController rotationDegreesPIDController,
	PPHolonomicDriveController pathPlannerHolonomicDriveController
) {

	public SwerveConstants(
		String logPath,
		double velocityAt12VoltsMetersPerSecond,
		Rotation2d maxRotationalVelocityPerSecond,
		PIDConstants translationMetersPIDConstants,
		PIDConstants rotationDegreesPIDConstants,
		PIDConstants PPTranslationMetersPIDConstants,
		PIDConstants PPRotationDegreesPIDConstants
	) {
		this(
			logPath,
			logPath + "/State",
			logPath + "/Velocity",
			velocityAt12VoltsMetersPerSecond,
			maxRotationalVelocityPerSecond,
			new PIDController(translationMetersPIDConstants.kP, translationMetersPIDConstants.kI, translationMetersPIDConstants.kD),
			new PIDController(translationMetersPIDConstants.kP, translationMetersPIDConstants.kI, translationMetersPIDConstants.kD),
			new PIDController(rotationDegreesPIDConstants.kP, rotationDegreesPIDConstants.kI, rotationDegreesPIDConstants.kD),
			new PPHolonomicDriveController(PPTranslationMetersPIDConstants, PPRotationDegreesPIDConstants)
		);
		this.xMetersPIDController.setIntegratorRange(-SwerveConstants.DEADBANDS.getX(), SwerveConstants.DEADBANDS.getX());
		this.yMetersPIDController.setIntegratorRange(-SwerveConstants.DEADBANDS.getY(), SwerveConstants.DEADBANDS.getY());
		this.rotationDegreesPIDController
			.setIntegratorRange(-SwerveConstants.DEADBANDS.getRotation().getDegrees(), SwerveConstants.DEADBANDS.getRotation().getDegrees());

		this.rotationDegreesPIDController
			.enableContinuousInput(MathConstants.HALF_CIRCLE.unaryMinus().getDegrees(), MathConstants.HALF_CIRCLE.getDegrees());
	}

	static final Rotation2d WHEEL_RADIUS_CALIBRATION_VELOCITY_PER_SECOND = Rotation2d.fromRotations(0.5);

	public static final double AIM_ASSIST_MAGNITUDE_FACTOR = 4;

	public static final Pose2d DEADBANDS = new Pose2d(0.0092, 0.0092, Rotation2d.fromRadians(0.0115));
	static final Rotation2d CALIBRATION_MODULE_ANGLE_TOLERANCE = Rotation2d.fromDegrees(3);
	static final Rotation2d CALIBRATION_MODULE_ANGULAR_VELOCITY_PER_SECOND_DEADBAND = Rotation2d.fromDegrees(3);

	public static final double MAX_DRIVE_ACCELERATION_METERS_PER_SECOND_SQ = 100;
	public static final double MAX_DRIVE_SKID_ACCELERATION_METERS_PER_SECOND_SQ = 100;
	public static final double MAX_DRIVE_TILT_ACCELERATION_METERS_PER_SECOND_SQ = 100;

	public static double getMaxLinearAcceleration() {
		return Math.min(
			Math.min(MAX_DRIVE_ACCELERATION_METERS_PER_SECOND_SQ, MAX_DRIVE_SKID_ACCELERATION_METERS_PER_SECOND_SQ),
			MAX_DRIVE_TILT_ACCELERATION_METERS_PER_SECOND_SQ
		);
	}

}
