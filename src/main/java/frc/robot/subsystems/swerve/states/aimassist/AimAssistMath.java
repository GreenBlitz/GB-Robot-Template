package frc.robot.subsystems.swerve.states.aimassist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.constants.field.Field;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveMath;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.utils.angle2d.Angle2d;
import frc.utils.math.FieldMath;
import frc.utils.math.ToleranceMath;

public class AimAssistMath {

	public static ChassisSpeeds getRotationAssistedSpeeds(
		ChassisSpeeds speeds,
		Angle2d robotHeading,
		Angle2d targetHeading,
		boolean slowDownByDriveMagnitude,
		SwerveConstants swerveConstants
	) {
		Angle2d pidOutputVelocityPerSecond = Angle2d
			.fromDegrees(swerveConstants.rotationDegreesPIDController().calculate(robotHeading.getDegrees(), targetHeading.getDegrees()));

		if (slowDownByDriveMagnitude) {
			pidOutputVelocityPerSecond = applyMagnitudeCompensation(pidOutputVelocityPerSecond, SwerveMath.getDriveMagnitude(speeds));
		}

		Angle2d clampedAngularVelocityPerSecond = ToleranceMath
			.clamp(pidOutputVelocityPerSecond, swerveConstants.maxRotationalVelocityPerSecond());

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
	public static ChassisSpeeds getObjectAssistedSpeedsSlowedDownByRotation(
		ChassisSpeeds speeds,
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

		ChassisSpeeds targetHeadingRelativeSpeeds = SwerveMath.allianceToRobotRelativeSpeeds(speeds, targetHeadingHingeSystemAngle);
		ChassisSpeeds assistedSpeeds = new ChassisSpeeds(
			targetHeadingRelativeSpeeds.vxMetersPerSecond,
			neededHorizontalVelocityRelativeToObjectMetersPerSecond,
			targetHeadingRelativeSpeeds.omegaRadiansPerSecond
		);

		if (slowDownByRotation) {
			assistedSpeeds.vxMetersPerSecond = applyMagnitudeCompensation(
				targetHeadingRelativeSpeeds.vxMetersPerSecond,
				Math.abs(speeds.omegaRadiansPerSecond),
				magnitudeCompensationFactor
			);
			assistedSpeeds.vyMetersPerSecond = applyMagnitudeCompensation(
				neededHorizontalVelocityRelativeToObjectMetersPerSecond,
				Math.abs(speeds.omegaRadiansPerSecond),
				magnitudeCompensationFactor
			);
		}

		return SwerveMath.robotToAllianceRelativeSpeeds(assistedSpeeds, targetHeadingHingeSystemAngle);
	}

	private static Angle2d applyMagnitudeCompensation(Angle2d velocityPerSecond, double magnitude) {
		return velocityPerSecond.times(SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR).div(magnitude + SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR);
	}

	private static double applyMagnitudeCompensation(double velocityPerSecond, double magnitude, double factor) {
		return velocityPerSecond * (SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR) / (magnitude + factor);
	}

}
