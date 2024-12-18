package frc.robot.vision.filters;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Field;
import frc.robot.vision.rawdata.AprilTagVisionData;
import frc.robot.vision.rawdata.VisionData;

public class VisionFilters {

	protected static Filter<VisionData> isPitchInTolerance(Rotation2d pitchTolerance) {
		return new Filter<>(rawVisionData -> Math.abs(rawVisionData.getEstimatedPose().getRotation().getY()) <= pitchTolerance.getRadians());
	}

	protected static Filter<VisionData> isRollInTolerance(Rotation2d rollTolerance) {
		return new Filter<>(rawVisionData -> Math.abs(rawVisionData.getEstimatedPose().getRotation().getX()) <= rollTolerance.getRadians());
	}

	protected static Filter<VisionData> isRobotOnGround(double robotToGroundToleranceMeters) {
		return new Filter<>(rawVisionData -> rawVisionData.getEstimatedPose().getZ() <= robotToGroundToleranceMeters);
	}

	protected static Filter<AprilTagVisionData> isAprilTagHeightInTolerance(
		double aprilTagHeightToleranceMeters,
		double aprilTagRealHeightMeters
	) {
		return new Filter<>(
			rawAprilTagVisionData -> Math.abs(rawAprilTagVisionData.getAprilTagHeight() - aprilTagRealHeightMeters)
				<= aprilTagHeightToleranceMeters
		);
	}

	protected static Filter<VisionData> isRobotXInField(double robotXPositionTolerance) {
		return new Filter<>(rawVisionData -> Math.abs(Field.WIDTH_METERS - rawVisionData.getEstimatedPose().getX()) <= robotXPositionTolerance);
	}

	protected static Filter<VisionData> isRobotYInField(double robotYPositionTolerance) {
		return new Filter<>(rawVisionData -> Math.abs(Field.LENGTH_METERS - rawVisionData.getEstimatedPose().getY()) <= robotYPositionTolerance);
	}

}
