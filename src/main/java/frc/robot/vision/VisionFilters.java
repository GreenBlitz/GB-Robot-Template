package frc.robot.vision;

import edu.wpi.first.math.geometry.*;
import frc.robot.vision.filters.Filter;
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

	//@formatter:off
	protected static Filter<AprilTagVisionData> isAprilTagHeightInTolerance(
			double aprilTagHeightToleranceMeters,
			double aprilTagRealHeightMeters
	) {
		return new Filter<>(
			rawAprilTagVisionData -> Math.abs(rawAprilTagVisionData.getAprilTagHeight() - aprilTagRealHeightMeters)
				<= aprilTagHeightToleranceMeters
		);
	}
	//@formatter:on

}
