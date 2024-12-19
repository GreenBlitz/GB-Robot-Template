package frc.robot.vision.filters;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Field;
import frc.robot.vision.rawdata.AprilTagVisionData;
import frc.robot.vision.rawdata.VisionData;

public class VisionFilters {

	public static Filter<VisionData> isPitchInTolerance(Rotation2d pitchTolerance) {
		return new Filter<>(visionData -> Math.abs(visionData.getEstimatedPose().getRotation().getY()) <= pitchTolerance.getRadians());
	}

	public static Filter<VisionData> isRollInTolerance(Rotation2d rollTolerance) {
		return new Filter<>(visionData -> Math.abs(visionData.getEstimatedPose().getRotation().getX()) <= rollTolerance.getRadians());
	}

	public static Filter<VisionData> isRobotOnGround(double robotToGroundToleranceMeters) {
		return new Filter<>(visionData -> visionData.getEstimatedPose().getZ() <= robotToGroundToleranceMeters);
	}

	public static Filter<AprilTagVisionData> isAprilTagHeightInTolerance(double aprilTagHeightToleranceMeters, double aprilTagRealHeightMeters) {
		return new Filter<>(
			aprilTagVisionData -> Math.abs(aprilTagVisionData.getAprilTagHeight() - aprilTagRealHeightMeters) <= aprilTagHeightToleranceMeters
		);
	}

	public static Filter<VisionData> isRobotXWithinFieldBorders(double robotXPositionTolerance) {
		return new Filter<>(visionData -> Math.abs(Field.WIDTH_METERS - visionData.getEstimatedPose().getX()) <= robotXPositionTolerance);
	}

	public static Filter<VisionData> isRobotYWithinFieldBorders(double robotYPositionTolerance) {
		return new Filter<>(visionData -> Math.abs(Field.LENGTH_METERS - visionData.getEstimatedPose().getY()) <= robotYPositionTolerance);
	}

	public static Filter<VisionData> isRobotWithinFieldBorders(double robotPositionTolerance) {
		return new Filter<>(
			visionData -> isRobotXWithinFieldBorders(robotPositionTolerance).and(isRobotYWithinFieldBorders(robotPositionTolerance))
				.apply(visionData)
		);
	}

}
