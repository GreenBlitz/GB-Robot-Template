package frc.robot.statemachine.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.utils.math.FieldMath;
import org.littletonrobotics.junction.Logger;

public class TargetChecks {

	private final Superstructure superstructure;
	private static final String isReadyToShootLogPath = "Statemachine/TargetChecks/IsReadyToShoot";

	public TargetChecks(Superstructure superstructure) {
		this.superstructure = superstructure;
	}

	private static boolean isWithinDistance(Translation2d robotPosition, double maxShootingDistanceFromTargetMeters, Translation2d closestGoal) {
		return robotPosition.getDistance(closestGoal) <= maxShootingDistanceFromTargetMeters;
	}

	private static boolean isInAngleRange(Translation2d robotPosition, Pose2d closestGoal, Rotation2d maxAngleFromCenter) {
		Rotation2d AngleBetweenRobotAndGoal = FieldMath.getRelativeTranslation(closestGoal, robotPosition).getAngle();
		return Math.abs(AngleBetweenRobotAndGoal.getDegrees()) <= maxAngleFromCenter.getDegrees();
	}

	private static boolean isAtHeading(Pose2d robotPose, Translation2d closestGoal, Rotation2d headingTolerance, Rotation2d currentTurretAngle) {
		Rotation2d wantedAngle = FieldMath.getRelativeTranslation(robotPose, closestGoal).getAngle();
		Rotation2d currentFieldRelativeTurretAngle = Rotation2d
			.fromDegrees((robotPose.getRotation().getDegrees() + currentTurretAngle.getDegrees()) % 360);
		return MathUtil.isNear(wantedAngle.getDegrees(), currentFieldRelativeTurretAngle.getDegrees(), headingTolerance.getDegrees());
	}

	private static boolean isFlywheelAtVelocity(
		Rotation2d wantedFlywheelVelocityRotation2dPerSecond,
		Rotation2d flywheelVelocityRotation2dPerSecond,
		Rotation2d flywheelVelocityToleranceRotation2dPerSecond
	) {
		return MathUtil.isNear(
			wantedFlywheelVelocityRotation2dPerSecond.getDegrees(),
			flywheelVelocityRotation2dPerSecond.getDegrees(),
			flywheelVelocityToleranceRotation2dPerSecond.getDegrees()
		);
	}

	private static boolean isHoodAtPositon(Rotation2d wantedPosition, Rotation2d hoodPosition, Rotation2d tolerance) {
		return MathUtil.isNear(wantedPosition.getDegrees(), hoodPosition.getDegrees(), tolerance.getDegrees());
	}

	public static boolean isReadyToShoot(
		Robot robot,
		Rotation2d wantedFlywheelVelocityRPS,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d wantedHoodPosition,
		Rotation2d hoodPositionTolerance,
		Rotation2d headingTolerance,
		Rotation2d maxAngleFromGoalCenter,
		Pose2d closestGoal,
		double maxShootingDistanceFromTargetMeters
	) {
		Pose2d robotPose = robot.getPoseEstimator().getEstimatedPose();
		Rotation2d turretHeading = robot.getTurret().getPosition();
		Rotation2d flywheelVelocityRotation2dPerSecond = robot.getFlyWheel().getVelocity();
		Rotation2d hoodPosition = robot.getHood().getPosition();

		boolean isWithinDistance = isWithinDistance(
			robotPose.getTranslation(),
			maxShootingDistanceFromTargetMeters,
			closestGoal.getTranslation()
		);
		Logger.recordOutput(isReadyToShootLogPath + "/isInDistance", isWithinDistance);

		boolean isInRange = isInAngleRange(robotPose.getTranslation(), closestGoal, maxAngleFromGoalCenter);
		Logger.recordOutput(isReadyToShootLogPath + "/isInRange", isInRange);

		boolean isAtHeading = isAtHeading(robotPose, closestGoal.getTranslation(), headingTolerance, turretHeading);
		Logger.recordOutput(isReadyToShootLogPath + "/isAthHeading", isAtHeading);

		boolean isFlywheelReadyToShoot = isFlywheelAtVelocity(
			wantedFlywheelVelocityRPS,
			flywheelVelocityRotation2dPerSecond,
			flywheelVelocityToleranceRPS
		);
		Logger.recordOutput(isReadyToShootLogPath + "/isFlywheelAtVelocity", isFlywheelReadyToShoot);

		boolean isHoodAtPosition = isHoodAtPositon(wantedHoodPosition, hoodPosition, hoodPositionTolerance);
		Logger.recordOutput(isReadyToShootLogPath + "/isHoodAtPositon", isHoodAtPosition);

		return isFlywheelReadyToShoot && isHoodAtPosition && isInRange && isWithinDistance && isAtHeading;
	}

}
