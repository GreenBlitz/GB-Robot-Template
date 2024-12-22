package frc.robot.vision.filters;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Field;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.data.VisionData;
import frc.utils.Filter;

public class VisionFilters {

	public static Filter<VisionData> isPitchInTolerance(Rotation2d pitchTolerance) {
		return new Filter<>(visionData -> Math.abs(visionData.getEstimatedPose().getRotation().getY()) <= pitchTolerance.getRadians());
	}

	public static Filter<VisionData> isRollInTolerance(Rotation2d rollTolerance) {
		return new Filter<>(visionData -> Math.abs(visionData.getEstimatedPose().getRotation().getX()) <= rollTolerance.getRadians());
	}

	public static Filter<VisionData> isRobotOnGround(double robotToGroundToleranceMeters) {
		return new Filter<>(visionData -> Math.abs(visionData.getEstimatedPose().getZ()) <= robotToGroundToleranceMeters);
	}

	public static Filter<AprilTagVisionData> isAprilTagHeightInTolerance(double aprilTagHeightToleranceMeters, double aprilTagRealHeightMeters) {
		return new Filter<>(
			aprilTagVisionData -> Math.abs(aprilTagVisionData.getAprilTagHeightMeters() - aprilTagRealHeightMeters)
				<= aprilTagHeightToleranceMeters
		);
	}

	public static Filter<VisionData> isRobotXWithinFieldBorders(double robotXPositionTolerance) {
		return new Filter<>(
			visionData -> (Field.LENGTH_METERS + robotXPositionTolerance) >= visionData.getEstimatedPose().getX()
				&& -robotXPositionTolerance <= visionData.getEstimatedPose().getX()
		);
	}

	public static Filter<VisionData> isRobotYWithinFieldBorders(double robotYPositionTolerance) {
		return new Filter<>(
			visionData -> (Field.WIDTH_METERS + robotYPositionTolerance) >= visionData.getEstimatedPose().getY()
				&& -robotYPositionTolerance <= visionData.getEstimatedPose().getY()
		);
	}

	public static Filter<VisionData> isRobotWithinFieldBorders(double robotPositionTolerance) {
		return new Filter<>(
			visionData -> isRobotXWithinFieldBorders(robotPositionTolerance).and(isRobotYWithinFieldBorders(robotPositionTolerance))
				.apply(visionData)
		);
	}

}
