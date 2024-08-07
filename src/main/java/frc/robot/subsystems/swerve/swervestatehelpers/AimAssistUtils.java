package frc.robot.subsystems.swerve.swervestatehelpers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveMath;


import static frc.robot.subsystems.swerve.SwerveMath.getDriveMagnitude;

public class AimAssistUtils {

	public static ChassisSpeeds getRotationAssistedSpeeds(
		ChassisSpeeds wantedSpeeds,
		Rotation2d currentRotation,
		Rotation2d targetRotation,
		SwerveConstants swerveConstants
	) {
		Rotation2d pidVelocity = Rotation2d.fromDegrees(
			swerveConstants.rotationDegreesPIDController().calculate(currentRotation.getDegrees(), targetRotation.getDegrees())
		);

		double angularVelocityRadians = applyMagnitudeCompensation(pidVelocity, getDriveMagnitude(wantedSpeeds));
		double combinedAngularVelocityRadians = angularVelocityRadians + wantedSpeeds.omegaRadiansPerSecond;
		Rotation2d clampedAngularVelocityPerSecond = SwerveMath.clampRotationalVelocity(
			Rotation2d.fromRadians(combinedAngularVelocityRadians),
			swerveConstants.maxRotationalVelocityPerSecond()
		);

		return new ChassisSpeeds(
			wantedSpeeds.vxMetersPerSecond,
			wantedSpeeds.vyMetersPerSecond,
			clampedAngularVelocityPerSecond.getRadians()
		);
	}

	public static double applyMagnitudeCompensation(Rotation2d velocityPerSecond, double magnitude) {
		return velocityPerSecond.getRadians()
			* SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR
			/ (magnitude + SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR);
	}

}
