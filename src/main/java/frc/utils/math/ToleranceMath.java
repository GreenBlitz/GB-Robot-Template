package frc.utils.math;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.Pose2d;
import frc.utils.costomUnits.Angle2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import frc.robot.subsystems.swerve.SwerveMath;

public class ToleranceMath {

	public static boolean isAtPose(
		Pose2d targetPose,
		Pose2d currentPose,
		ChassisVelocities currentVelocities,
		Pose2d tolerances,
		Pose2d deadbands
	) {
		return isNear(targetPose, currentPose, tolerances) && SwerveMath.isStill(currentVelocities, deadbands);
	}

	public static boolean isNear(Pose2d wantedPose, Pose2d pose, Pose2d tolerance) {
		return isNear(wantedPose.getTranslation(), pose.getTranslation(), tolerance.getTranslation())
			&& isNearWrapped(wantedPose.getRotation(), pose.getRotation(), tolerance.getRotation());
	}

	public static boolean isNear(Translation2d wantedTranslation, Translation2d translation, double toleranceMeters) {
		return translation.getDistance(wantedTranslation) <= toleranceMeters;
	}

	public static boolean isNear(Translation2d wantedTranslation, Translation2d translation, Translation2d tolerance) {
		return isNear(wantedTranslation.getX(), translation.getX(), tolerance.getX())
			&& isNear(wantedTranslation.getY(), translation.getY(), tolerance.getY());
	}

	public static boolean isNearWrapped(Angle2d wantedAngle, Angle2d angle, Angle2d tolerance) {
		return Math.abs(MathUtil.angleModulus(wantedAngle.minus(angle).getRadians())) <= tolerance.getRadians();
	}

	public static boolean isNear(double wanted, double actual, double tolerance) {
		return Math.abs(wanted - actual) <= tolerance;
	}

	public static boolean isInRange(double value, double min, double max, double tolerance) {
		return (min - tolerance) <= value && value <= (max + tolerance);
	}

	public static boolean isInRange(double value, double min, double max) {
		return isInRange(value, min, max, 0);
	}

	public static double applyDeadband(double value, double deadband) {
		return Math.abs(value) <= deadband ? 0 : value;
	}

	public static Angle2d clamp(Angle2d angle, Angle2d maxAngle) {
		return Angle2d.fromRadians(Math.clamp(angle.getRadians(), -maxAngle.getRadians(), maxAngle.getRadians()));
	}

	public static Angle2d clamp(Angle2d angle, Angle2d minAngle, Angle2d maxAngle) {
		return Angle2d.fromRadians(Math.clamp(angle.getRadians(), minAngle.getRadians(), maxAngle.getRadians()));
	}

}
