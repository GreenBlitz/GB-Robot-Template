package frc.robot.vision;

import edu.wpi.first.math.geometry.*;
import frc.robot.vision.filters.Filter;
import frc.robot.vision.rawdata.RawVisionData;

public class VisionFilters {

	protected static Filter<RawVisionData> isPitchInTolerance(Rotation2d pitchTolerance) {
		return new Filter<>((RawVisionData rawVisionData) -> Math.abs(rawVisionData.getEstimatedPose().getRotation().getY()) <= pitchTolerance.getRadians());
	}

	protected static Filter<RawVisionData> isRollInTolerance(Rotation2d rollTolerance) {
		return new Filter<>((RawVisionData rawVisionData) -> Math.abs(rawVisionData.getEstimatedPose().getRotation().getX()) <= rollTolerance.getRadians());
	}

	protected static Filter<RawVisionData> isRobotOnGround(double robotToGroundToleranceMeters) {
		return new Filter<>((RawVisionData rawVisionData) -> rawVisionData.getEstimatedPose().getZ() <= robotToGroundToleranceMeters);
	}

}
