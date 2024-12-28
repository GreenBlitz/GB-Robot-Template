package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Field;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.data.VisionData;
import frc.utils.Filter;
import frc.utils.ToleranceUtils;

public class VisionFilters {

	public static Filter<VisionData> isPitchInTolerance(Rotation2d pitchTolerance, Rotation2d wantedPitch) {
		return new Filter<>(
			visionData -> ToleranceUtils
				.isRotation2dInTolerance(pitchTolerance, Rotation2d.fromRadians(visionData.getEstimatedPose().getRotation().getY()), wantedPitch)
		);
	}

	public static Filter<VisionData> isRollInTolerance(Rotation2d rollTolerance, Rotation2d wantedRoll) {
		return new Filter<>(
			visionData -> ToleranceUtils
				.isRotation2dInTolerance(rollTolerance, Rotation2d.fromRadians(visionData.getEstimatedPose().getRotation().getX()), wantedRoll)
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

	public static Filter<VisionData> isXOnField(double xToleranceMeters) {
		return new Filter<>(
			visionData -> ToleranceUtils.isInRange(xToleranceMeters, visionData.getEstimatedPose().getX(), Field.LENGTH_METERS, 0)
		);
	}

	public static Filter<VisionData> isYOnField(double yToleranceMeters) {
		return new Filter<>(
			visionData -> ToleranceUtils.isInRange(yToleranceMeters, visionData.getEstimatedPose().getY(), Field.WIDTH_METERS, 0)
		);
	}

	public static Filter<VisionData> isOnField(double positionToleranceMeters) {
		return isXOnField(positionToleranceMeters).and(isYOnField(positionToleranceMeters));
	}

}
