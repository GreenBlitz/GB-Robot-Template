package frc.robot.subsystems.swerve.states.aimassist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveMath;
import frc.robot.subsystems.swerve.states.SwerveState;

public class AimAssistMath {

	public static ChassisSpeeds getRotationAssistedChassisSpeeds(
		ChassisSpeeds speeds,
		Rotation2d robotHeading,
		Rotation2d targetHeading,
		SwerveConstants swerveConstants
	) {
		Rotation2d pidOutputVelocityPerSecond = Rotation2d
			.fromDegrees(swerveConstants.rotationDegreesPIDController().calculate(robotHeading.getDegrees(), targetHeading.getDegrees()));

		Rotation2d rotationalVelocitySeconds = applyMagnitudeCompensation(pidOutputVelocityPerSecond, SwerveMath.getDriveMagnitude(speeds));
		Rotation2d combinedRotationalVelocitySeconds = Rotation2d
			.fromRadians(rotationalVelocitySeconds.getRadians() + speeds.omegaRadiansPerSecond);
		Rotation2d clampedAngularVelocityPerSecond = SwerveMath
			.clampRotationalVelocity(combinedRotationalVelocitySeconds, swerveConstants.maxRotationalVelocityPerSecond());

		return new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, clampedAngularVelocityPerSecond.getRadians());
	}

	public static ChassisSpeeds getObjectAssistedSpeeds(
		ChassisSpeeds speeds,
		Pose2d robotPose,
		Translation2d objectTranslation,
		SwerveConstants swerveConstants,
		SwerveState swerveState
	) {
		Translation2d objectRelativeToRobot = SwerveMath.getRelativeTranslation(robotPose, objectTranslation);
		double pidHorizontalToObjectOutputVelocityMetersPerSecond = swerveConstants.yMetersPIDController().calculate(0, objectRelativeToRobot.getY());
		double xVelocityMetersPerSecond = speeds.vxMetersPerSecond;
		double yVelocityMetersPerSecond = speeds.vyMetersPerSecond;

		switch (swerveState.getDriveMode()) {
			case FIELD_RELATIVE -> {
				double xFieldRelativeVelocityAddition = pidHorizontalToObjectOutputVelocityMetersPerSecond
					* robotPose.getRotation().unaryMinus().getSin();
				double yFieldRelativeVelocityAddition = pidHorizontalToObjectOutputVelocityMetersPerSecond
					* robotPose.getRotation().unaryMinus().getCos();

				xVelocityMetersPerSecond += xFieldRelativeVelocityAddition;
				yVelocityMetersPerSecond += yFieldRelativeVelocityAddition;
			}
			case ROBOT_RELATIVE -> {
				yVelocityMetersPerSecond += pidHorizontalToObjectOutputVelocityMetersPerSecond;
			}
		}

		return new ChassisSpeeds(xVelocityMetersPerSecond, yVelocityMetersPerSecond, speeds.omegaRadiansPerSecond);
	}

	public static Rotation2d applyMagnitudeCompensation(Rotation2d velocitySeconds, double magnitude) {
		return velocitySeconds.times(SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR).div(magnitude + SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR);
	}

}
