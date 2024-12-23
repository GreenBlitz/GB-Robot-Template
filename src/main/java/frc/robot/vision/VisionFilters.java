package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Field;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.data.VisionData;
import frc.utils.Filter;

public class VisionFilters {

	public static Filter<VisionData> isPitchInTolerance(Rotation2d pitchTolerance) {
		return new Filter<>(
			visionData -> Math.abs(visionData.getEstimatedPose().getRotation().getY()) <= pitchTolerance.getRadians()
				|| Math.abs(visionData.getEstimatedPose().getRotation().getY()) >= Math.toRadians(360) - pitchTolerance.getRadians()
		);
	}

	public static Filter<VisionData> isRollInTolerance(Rotation2d rollTolerance) {
		return new Filter<>(
			visionData -> Math.abs(visionData.getEstimatedPose().getRotation().getX()) <= rollTolerance.getRadians()
				|| Math.abs(visionData.getEstimatedPose().getRotation().getX()) >= Math.toRadians(360) - rollTolerance.getRadians()
		);
	}

	public static Filter<VisionData> isRobotOnGround(double robotToGroundToleranceMeters) {
		return new Filter<>(visionData -> Math.abs(visionData.getEstimatedPose().getZ()) <= robotToGroundToleranceMeters);
	}

	public static Filter<AprilTagVisionData> isAprilTagHeightInTolerance(double aprilTagHeightToleranceMeters, double aprilTagRealHeightMeters) {
		return new Filter<>(
			aprilTagVisionData -> Math.abs(aprilTagVisionData.getAprilTagHeightMeters() - aprilTagRealHeightMeters)
				<= aprilTagHeightToleranceMeters
		);
	}

	public static Filter<VisionData> isXWithinFieldBorders(double robotXPositionTolerance) {
		return new Filter<>(
			visionData -> (Field.LENGTH_METERS + robotXPositionTolerance) >= visionData.getEstimatedPose().getX()
				&& -robotXPositionTolerance <= visionData.getEstimatedPose().getX()
		);
	}

	public static Filter<VisionData> isYWithinFieldBorders(double robotYPositionTolerance) {
		return new Filter<>(
			visionData -> (Field.WIDTH_METERS + robotYPositionTolerance) >= visionData.getEstimatedPose().getY()
				&& -robotYPositionTolerance <= visionData.getEstimatedPose().getY()
		);
	}

	public static Filter<VisionData> isWithinFieldBorders(double robotPositionTolerance) {
		return new Filter<>(
			visionData -> isXWithinFieldBorders(robotPositionTolerance).and(isYWithinFieldBorders(robotPositionTolerance)).apply(visionData)
		);
	}

}
