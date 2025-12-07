package frc.robot.statemachine;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.utils.math.FieldMath;
import org.littletonrobotics.junction.Logger;

public class IsReadyHelperClass {

	private static final String isReadyToShootLogPath = "IsReadyToShoot";

	private static boolean isInDistanceForShooting(Translation2d robotPosition, double maxShootingRangeMeters, Translation2d closestGoal) {
		boolean isInRangeForShooting = robotPosition.getDistance(closestGoal) <= maxShootingRangeMeters;
		Logger.recordOutput(isReadyToShootLogPath + "/isInDistance", isInRangeForShooting);
		return isInRangeForShooting;
	}

	private static boolean isInRangeToShoot(Translation2d robotPosition, Pose2d closestGoal, Rotation2d maxAngleFromCenter) {
		Rotation2d AngleBetweenRobotAndGoal = FieldMath.getRelativeTranslation(closestGoal, robotPosition).getAngle();
		boolean isInRangeToShoot = Math.abs(AngleBetweenRobotAndGoal.getDegrees()) <= maxAngleFromCenter.getDegrees();
		Logger.recordOutput(isReadyToShootLogPath + "/isInRange", isInRangeToShoot);
		return isInRangeToShoot;
	}

	private static boolean isAtHeadingToShoot(
		Pose2d robotPose,
		Translation2d closestGoal,
		Rotation2d headingTolerance,
		Rotation2d turretHeading
	) {
		Rotation2d wantedAngle = FieldMath.getRelativeTranslation(robotPose, closestGoal).getAngle();
		Rotation2d shooterHeading = Rotation2d.fromDegrees((robotPose.getRotation().getDegrees() + turretHeading.getDegrees()) % 360);
		boolean isShooterAtHeading = MathUtil.isNear(wantedAngle.getDegrees(), shooterHeading.getDegrees(), headingTolerance.getDegrees());
		Logger.recordOutput(isReadyToShootLogPath + "/isAthHeading", isShooterAtHeading);
		return isShooterAtHeading;
	}

	private static boolean isAtPoseToShoot(
		Pose2d robotPose,
		Pose2d closestGoal,
		Rotation2d headingTolerance,
		Rotation2d maxAngleFromGoalCenter,
		Rotation2d turretHeading,
		double maxShootingRangeMeters
	) {
		return isInRangeToShoot(robotPose.getTranslation(), closestGoal, maxAngleFromGoalCenter)
			&& isAtHeadingToShoot(robotPose, closestGoal.getTranslation(), headingTolerance, turretHeading)
			&& isInDistanceForShooting(robotPose.getTranslation(), maxShootingRangeMeters, closestGoal.getTranslation());
	}

	private static boolean isFlywheelAtVelocity(
		Rotation2d wantedFlywheelVelocityRPS,
		Rotation2d flywheelVelocityRPS,
		Rotation2d flywheelVelocityToleranceRPS
	) {
		boolean isFlywheelAtVelocity = MathUtil
			.isNear(wantedFlywheelVelocityRPS.getDegrees(), flywheelVelocityRPS.getDegrees(), flywheelVelocityToleranceRPS.getDegrees());
		Logger.recordOutput(isReadyToShootLogPath + "/isFlywheelAtVelocity", isFlywheelAtVelocity);
		return isFlywheelAtVelocity;
	}

	private static boolean isHoodAtPositon(Rotation2d wantedPosition, Rotation2d hoodPosition, Rotation2d tolerance) {
		boolean isHoodAtPositon = MathUtil.isNear(wantedPosition.getDegrees(), hoodPosition.getDegrees(), tolerance.getDegrees());
		Logger.recordOutput(isReadyToShootLogPath + "/isHoodAtPositon", isHoodAtPositon);
		return isHoodAtPositon;
	}

	public static boolean isReadyToShoot(
		Rotation2d wantedFlywheelVelocityRPS,
		Rotation2d flywheelVelocityRPS,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d wantedHoodPosition,
		Rotation2d hoodPosition,
		Rotation2d hoodPositionTolerance,
		Rotation2d headingTolerance,
		Rotation2d maxAngleFromGoalCenter,
		Rotation2d turretHeading,
		Pose2d robotPose,
		Pose2d closestGoal,
		double maxShootingRangeMeters
	) {
		boolean isInPoseToShoot = isAtPoseToShoot(
			robotPose,
			closestGoal,
			headingTolerance,
			maxAngleFromGoalCenter,
			turretHeading,
			maxShootingRangeMeters
		);
		boolean isFlywheelReadyToShoot = isFlywheelAtVelocity(wantedFlywheelVelocityRPS, flywheelVelocityRPS, flywheelVelocityToleranceRPS);
		boolean isHoodAtPosition = isHoodAtPositon(wantedHoodPosition, hoodPosition, hoodPositionTolerance);
		return isFlywheelReadyToShoot && isHoodAtPosition && isInPoseToShoot;
	}


}
