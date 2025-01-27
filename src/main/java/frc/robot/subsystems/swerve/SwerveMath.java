package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.states.DriveSpeed;
import frc.utils.math.ToleranceMath;
import frc.utils.time.TimeUtils;

public class SwerveMath {

	public static double calculateDriveRadiusMeters(Translation2d[] modulePositionsFromCenterMeters) {
		double sum = 0;
		for (Translation2d modulePositionFromCenterMeters : modulePositionsFromCenterMeters) {
			sum += modulePositionFromCenterMeters.getDistance(new Translation2d());
		}
		return sum / modulePositionsFromCenterMeters.length;
	}

	public static ChassisSpeeds fieldToRobotRelativeSpeeds(ChassisSpeeds fieldRelativeSpeeds, Rotation2d allianceRelativeHeading) {
		return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, allianceRelativeHeading);
	}

	public static ChassisSpeeds robotToFieldRelativeSpeeds(ChassisSpeeds robotRelativeSpeeds, Rotation2d allianceRelativeHeading) {
		return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, allianceRelativeHeading);
	}

	public static ChassisSpeeds discretize(ChassisSpeeds chassisSpeeds) {
		return ChassisSpeeds.discretize(chassisSpeeds, TimeUtils.getLatestCycleTimeSeconds());
	}

	public static ChassisSpeeds powersToSpeeds(ChassisPowers powers, SwerveConstants constants) {
		return new ChassisSpeeds(
			powers.xPower() * constants.velocityAt12VoltsMetersPerSecond(),
			powers.yPower() * constants.velocityAt12VoltsMetersPerSecond(),
			powers.rotationalPower() * constants.maxRotationalVelocityPerSecond().getRadians()
		);
	}

	public static ChassisSpeeds factorSpeeds(ChassisSpeeds speeds, DriveSpeed driveSpeed) {
		return new ChassisSpeeds(
			speeds.vxMetersPerSecond * driveSpeed.getTranslationSpeedFactor(),
			speeds.vyMetersPerSecond * driveSpeed.getTranslationSpeedFactor(),
			speeds.omegaRadiansPerSecond * driveSpeed.getRotationSpeedFactor()
		);
	}

	public static ChassisSpeeds applyDeadband(ChassisSpeeds chassisSpeeds) {
		double xVelocityMetersPerSecond = ToleranceMath
			.applyDeadband(chassisSpeeds.vxMetersPerSecond, SwerveConstants.DRIVE_VELOCITY_METERS_PER_SECOND_DEADBAND);
		double yVelocityMetersPerSecond = ToleranceMath
			.applyDeadband(chassisSpeeds.vyMetersPerSecond, SwerveConstants.DRIVE_VELOCITY_METERS_PER_SECOND_DEADBAND);
		double rotationalVelocityRadiansPerSecond = ToleranceMath
			.applyDeadband(chassisSpeeds.omegaRadiansPerSecond, SwerveConstants.ROTATIONAL_VELOCITY_PER_SECOND_DEADBAND.getRadians());

		return new ChassisSpeeds(xVelocityMetersPerSecond, yVelocityMetersPerSecond, rotationalVelocityRadiansPerSecond);
	}

	public static boolean isStill(ChassisSpeeds chassisSpeeds) {
		return Math.abs(chassisSpeeds.vxMetersPerSecond) <= SwerveConstants.DRIVE_VELOCITY_METERS_PER_SECOND_DEADBAND
			&& Math.abs(chassisSpeeds.vyMetersPerSecond) <= SwerveConstants.DRIVE_VELOCITY_METERS_PER_SECOND_DEADBAND
			&& Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= SwerveConstants.ROTATIONAL_VELOCITY_PER_SECOND_DEADBAND.getRadians();
	}

	public static double getDriveMagnitude(ChassisSpeeds chassisSpeeds) {
		return Math.sqrt(Math.pow(chassisSpeeds.vxMetersPerSecond, 2) + Math.pow(chassisSpeeds.vyMetersPerSecond, 2));
	}

}
