package frc.robot.statemachine;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.utils.math.FieldMath;

public class IsReadyHelperClass {

	private static boolean isInRangeForShooting(Translation2d position, double maxShootingRangeMeters, Translation2d closestGoal) {
		return position.getDistance(closestGoal) <= maxShootingRangeMeters;
	}

	private static boolean isInPositionToShoot(
		Translation2d position,
		Pose2d closestGoal,
		Rotation2d maxAngleFromCenter,
		double maxShootingRangeMeters
	) {
		Rotation2d robotAngleComparedToGoal = FieldMath.getRelativeTranslation(closestGoal, position).getAngle();
		return Math.abs(robotAngleComparedToGoal.getDegrees()) <= maxAngleFromCenter.getDegrees()
			&& isInRangeForShooting(position, maxShootingRangeMeters, closestGoal.getTranslation());
	}

	private static boolean isInHeadingToShoot(Pose2d pose, Translation2d closestGoal, Rotation2d tolerance, Rotation2d turretPose) {
		Rotation2d wantedAngle = FieldMath.getRelativeTranslation(pose, closestGoal).getAngle();
		Rotation2d turretHeading = Rotation2d.fromDegrees(pose.getRotation().getDegrees() + turretPose.getDegrees());
		return MathUtil.isNear(wantedAngle.getDegrees(), turretHeading.getDegrees(), tolerance.getDegrees());
	}

	private static boolean isInPoseToShoot(
		Pose2d pose,
		Pose2d closestGoal,
		Rotation2d headingTolerance,
		Rotation2d maxAngleFromGoalCenter,
		Rotation2d turretPosition,
		double maxShootingRangeMeters
	) {
		return isInPositionToShoot(pose.getTranslation(), closestGoal, maxAngleFromGoalCenter, maxShootingRangeMeters)
			&& isInHeadingToShoot(pose, closestGoal.getTranslation(), headingTolerance, turretPosition);
	}

	private static boolean isFlywheelAtSpeed(Rotation2d wantedFlywheelVelocity, Rotation2d flywheelVelocity, Rotation2d tolerance) {
		return MathUtil.isNear(wantedFlywheelVelocity.getDegrees(), flywheelVelocity.getDegrees(), tolerance.getDegrees());
	}

	private static boolean isHoodAtPositon(Rotation2d wantedPosition, Rotation2d hoodPosition, Rotation2d tolerance) {
		return MathUtil.isNear(wantedPosition.getDegrees(), hoodPosition.getDegrees(), tolerance.getDegrees());
	}

	private static boolean isShooterReedyToShoot(
		Rotation2d wantedFlywheelVelocity,
		Rotation2d flywheelVelocity,
		Rotation2d shooterSpeedTolerance,
		Rotation2d wantedHoodPosition,
		Rotation2d hoodPosition,
		Rotation2d hoodPositionTolerance
	) {
		return isFlywheelAtSpeed(wantedFlywheelVelocity, flywheelVelocity, shooterSpeedTolerance)
			&& isHoodAtPositon(wantedHoodPosition, hoodPosition, hoodPositionTolerance);
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
		Rotation2d turretPosition,
		Pose2d pose,
		Pose2d closestGoal,
		double maxShootingRangeMeters
	) {
		boolean isInPoseToShoot = isInPoseToShoot(
			pose,
			closestGoal,
			headingTolerance,
			maxAngleFromGoalCenter,
			turretPosition,
			maxShootingRangeMeters
		);
		boolean isShooterReedyToShoot = isShooterReedyToShoot(
			wantedFlywheelVelocity,
			flywheelVelocity,
			shooterSpeedTolerance,
			wantedHoodPosition,
			hoodPosition,
			hoodPositionTolerance
		);
		return isShooterReedyToShoot && isInPoseToShoot;
	}


}
