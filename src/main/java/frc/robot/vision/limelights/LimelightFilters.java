package frc.robot.vision.limelights;

import edu.wpi.first.math.geometry.*;
import frc.robot.vision.RawVisionData;

public class LimelightFilters {

	//@formatter:off
	protected static boolean keepLimelightData(
		RawVisionData rawVisionData,
		LimelightFiltersTolerances tolerances
	) {
		return LimelightFilters.isRollInTolerance(rawVisionData, tolerances.rollTolerance())
			&& LimelightFilters.isPitchInTolerance(rawVisionData, tolerances.pitchTolerance())
			&& LimelightFilters.isRobotOnGround(rawVisionData, tolerances.robotToGroundToleranceMeters());
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
