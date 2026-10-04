package frc.robot.subsystems.swerve;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import frc.robot.subsystems.swerve.states.DriveSpeed;
import frc.utils.math.ToleranceMath;
import frc.utils.time.TimeUtil;

public class SwerveMath {

	public static double calculateDriveRadiusMeters(Translation2d[] modulePositionsFromCenterMeters) {
		double sum = 0;
		for (Translation2d modulePositionFromCenterMeters : modulePositionsFromCenterMeters) {
			sum += modulePositionFromCenterMeters.getDistance(new Translation2d());
		}
		return sum / modulePositionsFromCenterMeters.length;
	}

	public static ChassisVelocities allianceToRobotRelativeVelocities(
		ChassisVelocities allianceRelativeVelocities,
		Rotation2d allianceRelativeHeading
	) {
		return allianceRelativeVelocities.toRobotRelative(allianceRelativeHeading);
	}

	public static ChassisVelocities robotToAllianceRelativeVelocities(
		ChassisVelocities robotRelativeVelocities,
		Rotation2d allianceRelativeHeading
	) {
		return robotRelativeVelocities.toFieldRelative(allianceRelativeHeading);
	}

	public static ChassisVelocities discretize(ChassisVelocities chassisVelocities) {
		return chassisVelocities.discretize(TimeUtil.getLatestCycleTimeSeconds());
	}

	public static ChassisVelocities powersToVelocities(ChassisPowers powers, SwerveConstants constants) {
		return new ChassisVelocities(
			powers.xPower * constants.velocityAt12VoltsMetersPerSecond(),
			powers.yPower * constants.velocityAt12VoltsMetersPerSecond(),
			powers.rotationalPower * constants.maxRotationalVelocityPerSecond().getRadians()
		);
	}

	public static ChassisVelocities factorVelocities(ChassisVelocities velocities, DriveSpeed driveSpeed) {
		return new ChassisVelocities(
			velocities.vx * driveSpeed.getTranslationSpeedFactor(),
			velocities.vy * driveSpeed.getTranslationSpeedFactor(),
			velocities.omega * driveSpeed.getRotationSpeedFactor()
		);
	}

	public static ChassisVelocities applyDeadband(ChassisVelocities chassisVelocities, Pose2d deadbands) {
		double xVelocityMetersPerSecond = ToleranceMath.applyDeadband(chassisVelocities.vx, deadbands.getX());
		double yVelocityMetersPerSecond = ToleranceMath.applyDeadband(chassisVelocities.vy, deadbands.getY());
		double rotationalVelocityRadiansPerSecond = ToleranceMath.applyDeadband(chassisVelocities.omega, deadbands.getRotation().getRadians());

		return new ChassisVelocities(xVelocityMetersPerSecond, yVelocityMetersPerSecond, rotationalVelocityRadiansPerSecond);
	}

	public static boolean isStill(ChassisVelocities chassisVelocities, Pose2d deadbands) {
		return Math.abs(chassisVelocities.vx) <= deadbands.getX()
			&& Math.abs(chassisVelocities.vy) <= deadbands.getY()
			&& Math.abs(chassisVelocities.omega) <= deadbands.getRotation().getRadians();
	}

	public static double getDriveMagnitude(ChassisVelocities chassisVelocities) {
		return Math.sqrt(Math.pow(chassisVelocities.vx, 2) + Math.pow(chassisVelocities.vy, 2));
	}

}
