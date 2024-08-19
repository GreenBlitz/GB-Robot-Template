package frc.robot.subsystems.swerve.swervestatehelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveMath;
import frc.robot.subsystems.swerve.SwerveState;

import static frc.robot.subsystems.swerve.SwerveMath.getDriveMagnitude;

public class AimAssistMath {

	public static ChassisSpeeds getRotationAssistedChassisSpeeds(
		ChassisSpeeds speeds,
		Rotation2d robotHeading,
		Rotation2d targetHeading,
		SwerveConstants swerveConstants
	) {
		Rotation2d pidVelocityPerSecond = Rotation2d
			.fromDegrees(swerveConstants.rotationDegreesPIDController().calculate(robotHeading.getDegrees(), targetHeading.getDegrees()));

		double angularVelocityRadiansPerSecond = applyMagnitudeCompensation(pidVelocityPerSecond, getDriveMagnitude(speeds));
		double combinedAngularVelocityRadiansPerSecond = angularVelocityRadiansPerSecond + speeds.omegaRadiansPerSecond;
		Rotation2d clampedAngularVelocityPerSecond = SwerveMath.clampRotationalVelocity(
			Rotation2d.fromRadians(combinedAngularVelocityRadiansPerSecond),
			swerveConstants.maxRotationalVelocityPerSecond()
		);

		return new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, clampedAngularVelocityPerSecond.getRadians());
	}

	public static ChassisSpeeds getObjectAssistedSpeeds(
		ChassisSpeeds speeds,
		Pose2d robotPose,
		Translation2d objectTranslation,
		SwerveConstants swerveConstants,
		SwerveState swerveState
	) {
		Translation2d noteRelativeToRobot = SwerveMath.getRelativeTranslation(robotPose, objectTranslation);
		double pidHorizontalVelocityMetersPerSecond = swerveConstants.yMetersPIDController().calculate(0, noteRelativeToRobot.getY());
		double xVelocityMetersPerSecond = speeds.vxMetersPerSecond;
		double yVelocityMetersPerSecond = speeds.vyMetersPerSecond;

		switch (swerveState.getDriveMode()) {
			case FIELD_RELATIVE -> {
				double xFieldRelativeVelocityAddition = pidHorizontalVelocityMetersPerSecond
					* Math.sin(robotPose.getRotation().unaryMinus().getRadians());
				double yFieldRelativeVelocityAddition = pidHorizontalVelocityMetersPerSecond
					* Math.cos(robotPose.getRotation().unaryMinus().getRadians());

				xVelocityMetersPerSecond += xFieldRelativeVelocityAddition;
				yVelocityMetersPerSecond += yFieldRelativeVelocityAddition;
			}
			case ROBOT_RELATIVE -> {
				yVelocityMetersPerSecond += pidHorizontalVelocityMetersPerSecond;
			}
		}

		return new ChassisSpeeds(xVelocityMetersPerSecond, yVelocityMetersPerSecond, speeds.omegaRadiansPerSecond);
	}

	public static double applyMagnitudeCompensation(Rotation2d velocityPerSecond, double magnitude) {
		return velocityPerSecond.getRadians()
			* SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR
			/ (magnitude + SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR);
	}

}
