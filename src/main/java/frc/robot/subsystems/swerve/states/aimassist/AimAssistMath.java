package frc.robot.subsystems.swerve.states.aimassist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.constants.field.Field;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveMath;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.utils.math.FieldMath;
import frc.utils.math.ToleranceMath;

public class AimAssistMath {

	public static ChassisSpeeds getRotationAssistedSpeeds(
		ChassisSpeeds speeds,
		Rotation2d robotHeading,
		Rotation2d targetHeading,
		SwerveConstants swerveConstants
	) {
		Rotation2d pidOutputVelocityPerSecond = Rotation2d
			.fromDegrees(swerveConstants.rotationDegreesPIDController().calculate(robotHeading.getDegrees(), targetHeading.getDegrees()));

		Rotation2d angularVelocityPerSecond = applyMagnitudeCompensation(pidOutputVelocityPerSecond, SwerveMath.getDriveMagnitude(speeds));
		Rotation2d clampedAngularVelocityPerSecond = ToleranceMath
			.clamp(angularVelocityPerSecond, swerveConstants.maxRotationalVelocityPerSecond());

		return new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, clampedAngularVelocityPerSecond.getRadians());
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
		Rotation2d allianceRelativeTargetHeading,
		Translation2d objectTranslation,
		SwerveConstants swerveConstants,
		SwerveState swerveState
	) {
		Pose2d robotPoseWithTargetHeading = new Pose2d(robotPose.getX(), robotPose.getY(), allianceRelativeTargetHeading);
		Translation2d objectRelativeToRobot = FieldMath.getRelativeTranslation(robotPoseWithTargetHeading, objectTranslation);
		double neededObjectHorizontalVelocityMetersPerSecond = swerveConstants.yMetersPIDController().calculate(0, objectRelativeToRobot.getY());

		Rotation2d targetHeadingHingeSystemAngle = switch (swerveState.getDriveRelative()) {
			case ALLIANCE_RELATIVE -> Field.getAllianceRelative(allianceRelativeTargetHeading);
			case ROBOT_RELATIVE -> allianceRelativeTargetHeading.minus(robotPose.getRotation());
		};

		ChassisSpeeds targetHeadingRelativeSpeeds = SwerveMath.allianceToRobotRelativeSpeeds(speeds, targetHeadingHingeSystemAngle);
		ChassisSpeeds assistedSpeeds = new ChassisSpeeds(
			targetHeadingRelativeSpeeds.vxMetersPerSecond,
			neededObjectHorizontalVelocityMetersPerSecond,
			targetHeadingRelativeSpeeds.omegaRadiansPerSecond
		);
		return SwerveMath.robotToAllianceRelativeSpeeds(assistedSpeeds, targetHeadingHingeSystemAngle);
	}

	private static Rotation2d applyMagnitudeCompensation(Rotation2d velocityPerSecond, double magnitude) {
		return velocityPerSecond.times(SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR).div(magnitude + SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR);
	}

}
