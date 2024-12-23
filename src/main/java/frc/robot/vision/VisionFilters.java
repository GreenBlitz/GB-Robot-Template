package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Field;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.data.VisionData;
import frc.utils.Filter;
import frc.utils.ToleranceCalculations;

public class VisionFilters {

	public static Filter<VisionData> isPitchInTolerance(Rotation2d pitchTolerance, Rotation2d wantedPitch) {
		return new Filter<>(
			visionData -> ToleranceCalculations
				.isRotation2dInTolerance(pitchTolerance, visionData.getEstimatedPose().getY(), wantedPitch.getRadians())
		);
	}

	public static Filter<VisionData> isRollInTolerance(Rotation2d rollTolerance, Rotation2d wantedRoll) {
		return new Filter<>(
			visionData -> ToleranceCalculations
				.isRotation2dInTolerance(rollTolerance, visionData.getEstimatedPose().getX(), wantedRoll.getRadians())
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
			visionData -> ToleranceCalculations
				.isDoubleInRange(robotXPositionTolerance, visionData.getEstimatedPose().getX(), Field.LENGTH_METERS, 0)
		);
	}

	public static Filter<VisionData> isYWithinFieldBorders(double robotYPositionTolerance) {
		return new Filter<>(
			visionData -> ToleranceCalculations
				.isDoubleInRange(robotYPositionTolerance, visionData.getEstimatedPose().getY(), Field.WIDTH_METERS, 0)
		);
	}

	public static Filter<VisionData> isWithinFieldBorders(double robotPositionTolerance) {
		return isXWithinFieldBorders(robotPositionTolerance).and(isYWithinFieldBorders(robotPositionTolerance));
	}

}
