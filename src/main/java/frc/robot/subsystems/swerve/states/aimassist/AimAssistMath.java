package frc.robot.subsystems.swerve.states.aimassist;

import org.wpilib.math.geometry.Pose2d;
import frc.utils.costomUnits.Angle2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import frc.constants.field.Field;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveMath;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.utils.math.FieldMath;
import frc.utils.math.ToleranceMath;

public class AimAssistMath {

	public static ChassisVelocities getRotationAssistedVelocities(
		ChassisVelocities velocities,
		Angle2d robotHeading,
		Angle2d targetHeading,
		boolean slowDownByDriveMagnitude,
		SwerveConstants swerveConstants
	) {
		Angle2d pidOutputVelocityPerSecond = Angle2d
			.fromDegrees(swerveConstants.rotationDegreesPIDController().calculate(robotHeading.getDegrees(), targetHeading.getDegrees()));

		if (slowDownByDriveMagnitude) {
			pidOutputVelocityPerSecond = applyMagnitudeCompensation(pidOutputVelocityPerSecond, SwerveMath.getDriveMagnitude(velocities));
		}

		Angle2d angularVelocityPerSecond = applyMagnitudeCompensation(pidOutputVelocityPerSecond, SwerveMath.getDriveMagnitude(velocities));
		Angle2d clampedAngularVelocityPerSecond = ToleranceMath
			.clamp(pidOutputVelocityPerSecond, swerveConstants.maxRotationalVelocityPerSecond());

		return new ChassisVelocities(velocities.vx, velocities.vy, clampedAngularVelocityPerSecond.getRadians());
	}

	/**
	 * @formatter:off
	 * Returns {@link ChassisVelocities} that aligns you to the object.
	 * The returned chassis velocities will move you horizontally to the object so your current heading will point to it.
	 * Example (0 is object, R is robot, > is heading):
	 * Current Pose:              Ending Pose:
	 * |            0          |   |   R>       0          |
	 * |                       |   |                       |
	 * |   R>                  |   |                       |
	 * |                       |   |                       |
	 * @formatter:on
	 */
	public static ChassisVelocities getObjectAssistedVelocities(
		ChassisVelocities velocities,
		Pose2d robotPose,
		Angle2d allianceRelativeTargetHeading,
		Translation2d objectTranslation,
		SwerveConstants swerveConstants,
		SwerveState swerveState,
		double magnitudeCompensationFactor,
		boolean slowDownByRotation
	) {
		Pose2d robotPoseWithTargetHeading = new Pose2d(robotPose.getX(), robotPose.getY(), allianceRelativeTargetHeading.toRotation2d());
		Translation2d objectRelativeToRobot = FieldMath.getRelativeTranslation(robotPoseWithTargetHeading, objectTranslation);
		double neededHorizontalVelocityRelativeToObjectMetersPerSecond = swerveConstants.yMetersPIDController()
			.calculate(0, objectRelativeToRobot.getY());

		Angle2d targetHeadingHingeSystemAngle = switch (swerveState.getDriveRelative()) {
			case ALLIANCE_RELATIVE -> Field.getAllianceRelative(allianceRelativeTargetHeading);
			case ROBOT_RELATIVE -> allianceRelativeTargetHeading.minus(Angle2d.fromRotation2d(robotPose.getRotation()));
		};

		ChassisVelocities targetHeadingRelativeVelocities = SwerveMath
			.allianceToRobotRelativeVelocities(velocities, targetHeadingHingeSystemAngle);
		ChassisVelocities assistedVelocities = new ChassisVelocities(
			targetHeadingRelativeVelocities.vx,
			neededHorizontalVelocityRelativeToObjectMetersPerSecond,
			targetHeadingRelativeVelocities.omega
		);

		if (slowDownByRotation) {
			assistedVelocities.vx = applyMagnitudeCompensation(
				targetHeadingRelativeVelocities.vx,
				Math.abs(velocities.omega),
				magnitudeCompensationFactor
			);
			assistedVelocities.vy = applyMagnitudeCompensation(
				neededHorizontalVelocityRelativeToObjectMetersPerSecond,
				Math.abs(velocities.omega),
				magnitudeCompensationFactor
			);
		}

		return SwerveMath.robotToAllianceRelativeVelocities(assistedVelocities, targetHeadingHingeSystemAngle);
	}

	private static Angle2d applyMagnitudeCompensation(Angle2d velocityPerSecond, double magnitude) {
		return velocityPerSecond.times(SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR).div(magnitude + SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR);
	}

	private static double applyMagnitudeCompensation(double velocityPerSecond, double magnitude, double factor) {
		return velocityPerSecond * (SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR) / (magnitude + factor);
	}

}
