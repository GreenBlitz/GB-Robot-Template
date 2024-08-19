package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.swervestatehelpers.DriveSpeed;
import frc.utils.cycletime.CycleTimeUtils;

public class SwerveMath {

	//@formatter:off
	public static ChassisSpeeds fieldRelativeToRobotRelativeSpeeds(ChassisSpeeds fieldRelativeSpeeds, Rotation2d allianceRelativeHeading) {
		return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, allianceRelativeHeading);
	}
	//@formatter:on

	public static ChassisSpeeds robotRelativeToFieldRelativeSpeeds(ChassisSpeeds robotRelativeSpeeds, Rotation2d robotHeading) {
		return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, robotHeading);
	}

	public static ChassisSpeeds discretize(ChassisSpeeds chassisSpeeds) {
		return ChassisSpeeds.discretize(chassisSpeeds, CycleTimeUtils.getCurrentCycleTime());
	}

	//@formatter:off
	public static ChassisSpeeds powersToSpeeds(double xPower, double yPower, double thetaPower, DriveSpeed driveSpeed, SwerveConstants constants) {
		return new ChassisSpeeds(
			xPower * driveSpeed.translationSpeedFactor * constants.velocityAt12VoltsMetersPerSecond(),
			yPower * driveSpeed.translationSpeedFactor * constants.velocityAt12VoltsMetersPerSecond(),
			thetaPower * driveSpeed.rotationSpeedFactor * constants.maxRotationalVelocityPerSecond().getRadians()
		);
	}
	//@formatter:on

	public static ChassisSpeeds applyDeadband(ChassisSpeeds chassisSpeeds) {
		double newXSpeed = getDeadbandSpeed(chassisSpeeds.vxMetersPerSecond, SwerveConstants.DRIVE_NEUTRAL_DEADBAND);
		double newYSpeed = getDeadbandSpeed(chassisSpeeds.vyMetersPerSecond, SwerveConstants.DRIVE_NEUTRAL_DEADBAND);
		double newOmegaSpeed = getDeadbandSpeed(chassisSpeeds.omegaRadiansPerSecond, SwerveConstants.ROTATION_NEUTRAL_DEADBAND.getRadians());

		return new ChassisSpeeds(newXSpeed, newYSpeed, newOmegaSpeed);
	}

	public static boolean isStill(ChassisSpeeds chassisSpeeds) {
		return Math.abs(chassisSpeeds.vxMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND
			&& Math.abs(chassisSpeeds.vyMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND
			&& Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= SwerveConstants.ROTATION_NEUTRAL_DEADBAND.getRadians();
	}

	public static double getDriveMagnitude(ChassisSpeeds chassisSpeeds) {
		return Math.sqrt(Math.pow(chassisSpeeds.vxMetersPerSecond, 2) + Math.pow(chassisSpeeds.vyMetersPerSecond, 2));
	}

	public static double getDeadbandSpeed(double speed, double deadband) {
		return Math.abs(speed) <= deadband ? 0 : speed;
	}

	public static Translation2d getRelativeTranslation(Translation2d relativeTo, Translation2d toRelative) {
		return toRelative.minus(relativeTo);
	}

	public static Translation2d getRelativeTranslation(Pose2d relativeTo, Translation2d toRelative) {
		return getRelativeTranslation(relativeTo.getTranslation(), toRelative).rotateBy(relativeTo.getRotation().unaryMinus());
	}

	public static Rotation2d clampRotationalVelocity(Rotation2d velocity, Rotation2d maxRotationalVelocity) {
		return Rotation2d
			.fromRadians(MathUtil.clamp(velocity.getRadians(), -maxRotationalVelocity.getRadians(), maxRotationalVelocity.getRadians()));
	}

}
