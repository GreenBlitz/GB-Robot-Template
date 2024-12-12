package frc.robot.vision;

import edu.wpi.first.math.geometry.*;
import frc.robot.vision.filters.Filter;
import frc.robot.vision.rawdata.IRawVisionData;
import frc.robot.vision.rawdata.RawVisionData;

public class VisionFilters {

	protected static Filter isPitchInTolerance(Rotation2d pitchTolerance) {
		return new Filter(
			(IRawVisionData rawVisionData) -> Math.abs(rawVisionData.getEstimatedPose().getRotation().getY()) <= pitchTolerance.getRadians()
		);
	}

	protected static Filter isRollInTolerance(Rotation2d rollTolerance) {
		return new Filter(
			(IRawVisionData rawVisionData) -> Math.abs(rawVisionData.getEstimatedPose().getRotation().getX()) <= rollTolerance.getRadians()
		);
	}

	protected static Filter isRobotOnGround(double robotToGroundToleranceMeters) {
		return new Filter((IRawVisionData rawVisionData) -> rawVisionData.getEstimatedPose().getZ() <= robotToGroundToleranceMeters);
	}

}
