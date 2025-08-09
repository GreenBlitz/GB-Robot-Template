package frc.robot.newvision.cameras.limelight;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.Field;
import frc.robot.poseestimator.helpers.robotheadingestimator.RobotHeadingEstimator;
import frc.utils.LimelightHelpers;
import frc.utils.filter.Filter;
import frc.utils.math.ToleranceMath;

import java.util.Optional;
import java.util.function.Supplier;

public class LimelightFilters {

	public static Filter megaTag1Filter(Limelight limelight, Translation2d robotInFieldTolerance) {
		LimelightHelpers.PoseEstimate poseEstimate = limelight.getMegaTag1RobotPoseEstimate();
		return isRobotInField(poseEstimate.pose::getTranslation, robotInFieldTolerance);
	}

	public static Filter megaTag2Filter(
		Limelight limelight,
		RobotHeadingEstimator headingEstimator,
		Translation2d robotInFieldTolerance,
		Rotation2d yawAtAngleTolerance
	) {
		LimelightHelpers.PoseEstimate poseEstimate = limelight.getMegaTag2RobotPoseEstimate();
		return isRobotInField(poseEstimate.pose::getTranslation, robotInFieldTolerance)
			.and(
				isYawAtAngle(
					poseEstimate.pose::getRotation,
					() -> headingEstimator.getEstimatedHeadingAtTimestamp(Limelight.getEstimateTimestampSeconds(poseEstimate)),
					yawAtAngleTolerance
				)
			)
			.and(isYawNotZero(poseEstimate.pose::getRotation));
	}

	public static Filter isYawAtAngle(Supplier<Rotation2d> robotYaw, Supplier<Optional<Rotation2d>> wantedYawSupplier, Rotation2d yawTolerance) {
		return () -> wantedYawSupplier.get()
			.map(wantedAngle -> ToleranceMath.isNearWrapped(wantedAngle, robotYaw.get(), yawTolerance))
			.orElse(false);
	}

	private static Filter isYawNotZero(Supplier<Rotation2d> robotYaw) {
		return () -> robotYaw.get().getRotations() != 0.0;
	}

	private static Filter isXInField(Supplier<Double> robotX, double xToleranceMeters) {
		return () -> ToleranceMath.isInRange(robotX.get(), 0, Field.LENGTH_METERS, xToleranceMeters);
	}

	private static Filter isYInField(Supplier<Double> robotY, double yToleranceMeters) {
		return () -> ToleranceMath.isInRange(robotY.get(), 0, Field.WIDTH_METERS, yToleranceMeters);
	}

	private static Filter isRobotInField(Supplier<Translation2d> robotTranslation, Translation2d tolerance) {
		return isXInField(robotTranslation.get()::getX, tolerance.getX()).and(isYInField(robotTranslation.get()::getY, tolerance.getY()));
	}

}
