package frc.robot.subsystems.swerve.swervestatehelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveMath;


import java.util.Optional;
import java.util.function.Supplier;

import static frc.robot.subsystems.swerve.SwerveMath.getDriveMagnitude;

public class AimAssistMath {

	public static ChassisSpeeds getRotationAssistedChassisSpeeds(
		ChassisSpeeds chassisSpeeds,
		Rotation2d currentRotation,
		Rotation2d targetRotation,
		SwerveConstants swerveConstants
	) {
		Rotation2d pidVelocity = Rotation2d
			.fromDegrees(swerveConstants.rotationDegreesPIDController().calculate(currentRotation.getDegrees(), targetRotation.getDegrees()));

		double angularVelocityRadians = applyMagnitudeCompensation(pidVelocity, getDriveMagnitude(chassisSpeeds));
		double combinedAngularVelocityRadians = angularVelocityRadians + chassisSpeeds.omegaRadiansPerSecond;
		Rotation2d clampedAngularVelocityPerSecond = SwerveMath
			.clampRotationalVelocity(Rotation2d.fromRadians(combinedAngularVelocityRadians), swerveConstants.maxRotationalVelocityPerSecond());

		return new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, clampedAngularVelocityPerSecond.getRadians());
	}
	
	public static ChassisSpeeds getObjectAssistedSpeeds (
			ChassisSpeeds chassisSpeeds,
			Pose2d robotPose,
			Translation2d objectTranslation,
			SwerveConstants swerveConstants
			){
		
		
		Translation2d noteRelativeToRobot = SwerveMath.getRelativeTranslation(robotPose, objectTranslation);
		
		double pidHorizontalVelocity = swerveConstants.yMetersPIDController().calculate(0, noteRelativeToRobot.getY());
		double xFieldRelativeVelocityAddition = pidHorizontalVelocity
				* Math.sin(robotPose.getRotation().unaryMinus().getRadians());
		double yFieldRelativeVelocityAddition = pidHorizontalVelocity
				* Math.cos(robotPose.getRotation().unaryMinus().getRadians());
		
		return new ChassisSpeeds(
				chassisSpeeds.vxMetersPerSecond + xFieldRelativeVelocityAddition,
				chassisSpeeds.vyMetersPerSecond + yFieldRelativeVelocityAddition,
				chassisSpeeds.omegaRadiansPerSecond
		);
	}

	public static double applyMagnitudeCompensation(Rotation2d velocityPerSecond, double magnitude) {
		return velocityPerSecond.getRadians()
			* SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR
			/ (magnitude + SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR);
	}

}
