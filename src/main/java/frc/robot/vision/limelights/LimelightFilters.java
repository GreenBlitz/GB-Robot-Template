package frc.robot.vision.limelights;

import edu.wpi.first.math.geometry.*;
import frc.robot.vision.VisionRawData;

public class LimelightFilters {

	//@formatter:off
	protected static boolean keepLimelightData(
		VisionRawData limelightRawData,
		LimelightFiltersTolerances tolerances
	) {
		return LimelightFilters.isRollInTolerance(limelightRawData, tolerances.rollTolerance())
			&& LimelightFilters.isPitchInTolerance(limelightRawData, tolerances.pitchTolerance())
			&& LimelightFilters.isRobotOnGround(limelightRawData, tolerances.robotToGroundToleranceMeters());
	}
	//@formatter:on

	protected static boolean isPitchInTolerance(VisionRawData limelightRawData, Rotation2d pitchTolerance) {
		return Math.abs(limelightRawData.estimatedPose().getRotation().getY()) <= pitchTolerance.getRadians();
	}

	protected static boolean isRollInTolerance(VisionRawData limelightRawData, Rotation2d rollTolerance) {
		return Math.abs(limelightRawData.estimatedPose().getRotation().getX()) <= rollTolerance.getRadians();
	}

	protected static boolean isRobotOnGround(VisionRawData limelightRawData, double robotToGroundToleranceMeters) {
		return limelightRawData.estimatedPose().getZ() <= robotToGroundToleranceMeters;
	}

}
