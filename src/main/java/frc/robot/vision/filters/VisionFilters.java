package frc.robot.vision.filters;

import edu.wpi.first.math.geometry.*;
import frc.robot.vision.filters.Filter;
import frc.robot.vision.rawdata.RawAprilTagVisionData;
import frc.robot.vision.rawdata.RawVisionData;

public class VisionFilters {

	protected static Filter<RawVisionData> isPitchInTolerance(Rotation2d pitchTolerance) {
		return new Filter<>(rawVisionData -> Math.abs(rawVisionData.getEstimatedPose().getRotation().getY()) <= pitchTolerance.getRadians());
	}

	protected static Filter<RawVisionData> isRollInTolerance(Rotation2d rollTolerance) {
		return new Filter<>(rawVisionData -> Math.abs(rawVisionData.getEstimatedPose().getRotation().getX()) <= rollTolerance.getRadians());
	}

	protected static Filter<RawVisionData> isRobotOnGround(double robotToGroundToleranceMeters) {
		return new Filter<>(rawVisionData -> rawVisionData.getEstimatedPose().getZ() <= robotToGroundToleranceMeters);
	}

	//@formatter:off
	protected static Filter<RawAprilTagVisionData> isAprilTagHeightInTolerance(
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
