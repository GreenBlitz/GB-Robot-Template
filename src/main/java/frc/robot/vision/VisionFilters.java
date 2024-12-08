package frc.robot.vision;

import edu.wpi.first.math.geometry.*;
import frc.robot.vision.rawdata.RawVisionData;

public class VisionFilters {

	public static Filter doesRawDataPassAllFilters(RawVisionData rawVisionData, VisionFiltersTolerances tolerances) {
		return VisionFilters.isRollInTolerance(rawVisionData, tolerances.rollTolerance())
			&& VisionFilters.isPitchInTolerance(rawVisionData, tolerances.pitchTolerance())
			&& VisionFilters.isRobotOnGround(rawVisionData, tolerances.robotDistanceFromGroundToleranceMeters());
	}

	protected static Filter isPitchInTolerance(RawVisionData rawVisionData, Rotation2d pitchTolerance) {
		return Math.abs(rawVisionData.getEstimatedPose().getRotation().getY()) <= pitchTolerance.getRadians();
	}

	protected static Filter isRollInTolerance(RawVisionData rawVisionData, Rotation2d rollTolerance) {
		return Math.abs(rawVisionData.getEstimatedPose().getRotation().getX()) <= rollTolerance.getRadians();
	}

	protected static Filter isRobotOnGround(RawVisionData rawVisionData, double robotToGroundToleranceMeters) {
		return rawVisionData.getEstimatedPose().getZ() <= robotToGroundToleranceMeters;
	}

}
