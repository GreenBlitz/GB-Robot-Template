package frc.robot.statemachine;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.utils.math.FieldMath;

public class IsReadyHelperClass {

	private static boolean isInRangeForShooting(Translation2d robotPosition, double maxShootingRangeMeters, Translation2d closestGoal) {
		return robotPosition.getDistance(closestGoal) <= maxShootingRangeMeters;
	}

	private static boolean isAtPositionToShoot(
		Translation2d robotPosition,
		Pose2d closestGoal,
		Rotation2d maxAngleFromCenter,
		double maxShootingRangeMeters
	) {
		Rotation2d AngleBetweenRobotAndGoal = FieldMath.getRelativeTranslation(closestGoal, robotPosition).getAngle();
		return Math.abs(AngleBetweenRobotAndGoal.getDegrees()) <= maxAngleFromCenter.getDegrees()
			&& isInRangeForShooting(robotPosition, maxShootingRangeMeters, closestGoal.getTranslation());
	}

	private static boolean isAtHeadingToShoot(
		Pose2d robotPose,
		Translation2d closestGoal,
		Rotation2d headingTolerance,
		Rotation2d turretHeading
	) {
		Rotation2d wantedAngle = FieldMath.getRelativeTranslation(robotPose, closestGoal).getAngle();
		Rotation2d shooterHeading = Rotation2d.fromDegrees(robotPose.getRotation().getDegrees() + turretHeading.getDegrees());
		return MathUtil.isNear(wantedAngle.getDegrees(), shooterHeading.getDegrees(), headingTolerance.getDegrees());
	}

	private static boolean isAtPoseToShoot(
		Pose2d robotPose,
		Pose2d closestGoal,
		Rotation2d headingTolerance,
		Rotation2d maxAngleFromGoalCenter,
		Rotation2d turretHeading,
		double maxShootingRangeMeters
	) {
		return isAtPositionToShoot(robotPose.getTranslation(), closestGoal, maxAngleFromGoalCenter, maxShootingRangeMeters)
			&& isAtHeadingToShoot(robotPose, closestGoal.getTranslation(), headingTolerance, turretHeading);
	}

	private static boolean isFlywheelAtSpeed(Rotation2d wantedFlywheelVelocity, Rotation2d flywheelVelocity, Rotation2d tolerance) {
		return MathUtil.isNear(wantedFlywheelVelocity.getDegrees(), flywheelVelocity.getDegrees(), tolerance.getDegrees());
	}

	private static boolean isHoodAtPositon(Rotation2d wantedPosition, Rotation2d hoodPosition, Rotation2d tolerance) {
		return MathUtil.isNear(wantedPosition.getDegrees(), hoodPosition.getDegrees(), tolerance.getDegrees());
	}

	public static boolean isReadyToShoot(
		Rotation2d wantedFlywheelVelocity,
		Rotation2d flywheelVelocity,
		Rotation2d shooterSpeedTolerance,
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
		boolean isShooterReedyToShoot = isFlywheelAtSpeed(wantedFlywheelVelocity, flywheelVelocity, shooterSpeedTolerance)
			&& isHoodAtPositon(wantedHoodPosition, hoodPosition, hoodPositionTolerance);
		return isShooterReedyToShoot && isInPoseToShoot;
	}


}
