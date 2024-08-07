package frc.robot.subsystems.swerve.swervestatehelpers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveMath;


import static frc.robot.subsystems.swerve.SwerveMath.getDriveMagnitude;

public class AimAssistUtils {

	public static ChassisSpeeds getRotationAssistedChassisSpeeds(
		ChassisSpeeds chassisSpeeds,
		Rotation2d currentRotation,
		Rotation2d targetRotation,
		SwerveConstants swerveConstants
	) {
		Rotation2d pidVelocity = Rotation2d.fromDegrees(
			swerveConstants.rotationDegreesPIDController().calculate(currentRotation.getDegrees(), targetRotation.getDegrees())
		);

		double angularVelocityRadians = applyMagnitudeCompensation(pidVelocity, getDriveMagnitude(chassisSpeeds));
		double combinedAngularVelocityRadians = angularVelocityRadians + chassisSpeeds.omegaRadiansPerSecond;
		Rotation2d clampedAngularVelocityPerSecond = SwerveMath.clampRotationalVelocity(
			Rotation2d.fromRadians(combinedAngularVelocityRadians),
			swerveConstants.maxRotationalVelocityPerSecond()
		);

		return new ChassisSpeeds(
			chassisSpeeds.vxMetersPerSecond,
			chassisSpeeds.vyMetersPerSecond,
			clampedAngularVelocityPerSecond.getRadians()
		);
	}

	public static double applyMagnitudeCompensation(Rotation2d velocityPerSecond, double magnitude) {
		return velocityPerSecond.getRadians()
			* SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR
			/ (magnitude + SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR);
	}

}
