package frc.robot.subsystems.swerve.states.aimassist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.constants.field.Field;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveMath;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.utils.math.PoseMath;
import frc.utils.math.ToleranceMath;

public class AimAssistMath {

	public static ChassisSpeeds getRotationAssistedChassisSpeeds(
		ChassisSpeeds speeds,
		Rotation2d robotHeading,
		Rotation2d targetHeading,
		SwerveConstants swerveConstants
	) {
		Rotation2d pidOutputVelocityPerSecond = Rotation2d
			.fromDegrees(swerveConstants.rotationDegreesPIDController().calculate(robotHeading.getDegrees(), targetHeading.getDegrees()));

		Rotation2d rotationalVelocityPerSecond = applyMagnitudeCompensation(pidOutputVelocityPerSecond, SwerveMath.getDriveMagnitude(speeds));
		Rotation2d combinedRotationalVelocityPerSecond = Rotation2d
			.fromRadians(rotationalVelocityPerSecond.getRadians() + speeds.omegaRadiansPerSecond);
		Rotation2d clampedRotationalVelocityPerSecond = ToleranceMath
			.clamp(combinedRotationalVelocityPerSecond, swerveConstants.maxRotationalVelocityPerSecond());

		return new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, clampedRotationalVelocityPerSecond.getRadians());
	}

	/**
	 * @formatter:off
	 * Returns {@link ChassisSpeeds} that aligns you to the object.
	 * The returned chassis speeds will move you horizontally to the object so your current heading will point to it.
	 * Example (0 is object, R is robot, > is heading):
	 * Current Pose:              Ending Pose:
	 * |            0          |   |   R>       0          |
	 * |                       |   |                       |
	 * |   R>                  |   |                       |
	 * |                       |   |                       |
	 * @formatter:on
	 */
	public static ChassisSpeeds getObjectAssistedSpeeds(
		ChassisSpeeds speeds,
		Pose2d robotPose,
		Translation2d objectTranslation,
		SwerveConstants swerveConstants,
		SwerveState swerveState
	) {
		Translation2d objectRelativeToRobot = PoseMath.getRelativeTranslation(robotPose, objectTranslation);
		double pidHorizontalToObjectOutputVelocityMetersPerSecond = swerveConstants.yMetersPIDController()
			.calculate(0, objectRelativeToRobot.getY());
		double xVelocityMetersPerSecond = speeds.vxMetersPerSecond;
		double yVelocityMetersPerSecond = speeds.vyMetersPerSecond;

		switch (swerveState.getDriveMode()) {
			case FIELD_RELATIVE -> {
				double xFieldRelativeVelocityAddition = pidHorizontalToObjectOutputVelocityMetersPerSecond
					* robotPose.getRotation().unaryMinus().getSin();
				double yFieldRelativeVelocityAddition = pidHorizontalToObjectOutputVelocityMetersPerSecond
					* robotPose.getRotation().unaryMinus().getCos();

				if (!Field.isFieldConventionAlliance()) {
					xFieldRelativeVelocityAddition = -xFieldRelativeVelocityAddition;
					yFieldRelativeVelocityAddition = -yFieldRelativeVelocityAddition;
				}

				xVelocityMetersPerSecond += xFieldRelativeVelocityAddition;
				yVelocityMetersPerSecond += yFieldRelativeVelocityAddition;
			}
			case ROBOT_RELATIVE -> {
				if (!Field.isFieldConventionAlliance()) {
					pidHorizontalToObjectOutputVelocityMetersPerSecond = -pidHorizontalToObjectOutputVelocityMetersPerSecond;
				}
				yVelocityMetersPerSecond += pidHorizontalToObjectOutputVelocityMetersPerSecond;
			}
		}

		return new ChassisSpeeds(xVelocityMetersPerSecond, yVelocityMetersPerSecond, speeds.omegaRadiansPerSecond);
	}

	public static Rotation2d applyMagnitudeCompensation(Rotation2d velocityPerSecond, double magnitude) {
		return velocityPerSecond.times(SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR).div(magnitude + SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR);
	}

}
