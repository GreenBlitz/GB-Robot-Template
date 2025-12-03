package frc.robot.statemachine;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.utils.math.FieldMath;

public class IsReady {

	private static boolean isInRangeForShouting(Translation2d position, double maxShoutingRangeMeters, Translation2d closestGoal) {
		return position.getDistance(closestGoal) <= maxShoutingRangeMeters;
	}

	private static boolean isInPositionToShout(
		Translation2d position,
		Pose2d closestGoal,
		Rotation2d maxAngleFromCenter,
		double maxShoutingRangeMeters
	) {
		Rotation2d robotAngleComperedToGoal = FieldMath.getRelativeTranslation(closestGoal, position).getAngle();
		return Math.abs(robotAngleComperedToGoal.getDegrees()) <= maxAngleFromCenter.getDegrees()
			&& isInRangeForShouting(position, maxShoutingRangeMeters, closestGoal.getTranslation());
	}

	private static boolean isInHeadingToShout(Pose2d pose, Translation2d closestGoal, Rotation2d tolerance, Rotation2d turretPose) {
		Rotation2d wantedAngle = FieldMath.getRelativeTranslation(pose, closestGoal).getAngle();
		Rotation2d turretHeading = Rotation2d.fromDegrees(pose.getRotation().getDegrees() + turretPose.getDegrees());
		return MathUtil.isNear(wantedAngle.getDegrees(), turretHeading.getDegrees(), tolerance.getDegrees());
	}

	private static boolean isInPoseToShout(
		Pose2d pose,
		Pose2d closestGoal,
		Rotation2d headingTolerance,
		Rotation2d maxAngleFromGoalCenter,
		Rotation2d turretPosition,
		double maxShoutingRangeMeters
	) {
		return isInPositionToShout(pose.getTranslation(), closestGoal, maxAngleFromGoalCenter, maxShoutingRangeMeters)
			&& isInHeadingToShout(pose, closestGoal.getTranslation(), headingTolerance, turretPosition);
	}

	private static boolean isFlywheelAtSpeed(Rotation2d wantedFlywheelVelocity, Rotation2d flywheelVelocity, Rotation2d tolerance) {
		return MathUtil.isNear(wantedFlywheelVelocity.getDegrees(), flywheelVelocity.getDegrees(), tolerance.getDegrees());
	}

	private static boolean isHoodAtPositon(Rotation2d wantedPosition, Rotation2d hoodPosition, Rotation2d tolerance) {
		return MathUtil.isNear(wantedPosition.getDegrees(), hoodPosition.getDegrees(), tolerance.getDegrees());
	}

	private static boolean isShouterReedyToShout(
		Rotation2d wantedFlywheelVelocity,
		Rotation2d flywheelVelocity,
		Rotation2d shouterSpeedTolerance,
		Rotation2d wantedHoodPosition,
		Rotation2d hoodPosition,
		Rotation2d hoodPositionTolerance
	) {
		return isFlywheelAtSpeed(wantedFlywheelVelocity, flywheelVelocity, shouterSpeedTolerance)
			&& isHoodAtPositon(wantedHoodPosition, hoodPosition, hoodPositionTolerance);
	}

	public static boolean isReadyToShout(
		Rotation2d wantedFlywheelVelocity,
		Rotation2d flywheelVelocity,
		Rotation2d shouterSpeedTolerance,
		Rotation2d wantedHoodPosition,
		Rotation2d hoodPosition,
		Rotation2d hoodPositionTolerance,
		Rotation2d headingTolerance,
		Rotation2d maxAngleFromGoalCenter,
		Rotation2d turretPosition,
		Pose2d pose,
		Pose2d closestGoal,
		double maxShoutingRangeMeters
	) {
		boolean isInPoseToShout = isInPoseToShout(
			pose,
			closestGoal,
			headingTolerance,
			maxAngleFromGoalCenter,
			turretPosition,
			maxShoutingRangeMeters
		);
		boolean isShouterReedyToShout = isShouterReedyToShout(
			wantedFlywheelVelocity,
			flywheelVelocity,
			shouterSpeedTolerance,
			wantedHoodPosition,
			hoodPosition,
			hoodPositionTolerance
		);
		return isShouterReedyToShout && isInPoseToShout;
	}

}
