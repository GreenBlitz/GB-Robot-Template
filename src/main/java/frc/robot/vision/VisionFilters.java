package frc.robot.vision;

import edu.wpi.first.math.geometry.*;

public class VisionFilters {

	//@formatter:off
	public static boolean keepVisionData(
		RawVisionData rawVisionData,
		VisionFiltersTolerances tolerances
	) {
		return VisionFilters.isRollInTolerance(rawVisionData, tolerances.rollTolerance())
			&& VisionFilters.isPitchInTolerance(rawVisionData, tolerances.pitchTolerance())
			&& VisionFilters.isRobotOnGround(rawVisionData, tolerances.robotToGroundToleranceMeters());
	}
	//@formatter:on

	protected static boolean isPitchInTolerance(RawVisionData rawVisionData, Rotation2d pitchTolerance) {
		return Math.abs(rawVisionData.estimatedPose().getRotation().getY()) <= pitchTolerance.getRadians();
	}

	protected static boolean isRollInTolerance(RawVisionData rawVisionData, Rotation2d rollTolerance) {
		return Math.abs(rawVisionData.estimatedPose().getRotation().getX()) <= rollTolerance.getRadians();
	}

	protected static boolean isRobotOnGround(RawVisionData rawVisionData, double robotToGroundToleranceMeters) {
		return rawVisionData.estimatedPose().getZ() <= robotToGroundToleranceMeters;
	}

}
