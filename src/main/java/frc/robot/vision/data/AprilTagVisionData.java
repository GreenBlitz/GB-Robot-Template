package frc.robot.vision.data;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.vision.sources.VisionSource;
import frc.utils.math.StandardDeviations3D;

public class AprilTagVisionData extends VisionData {

	private final StandardDeviations3D standardDeviations;
	private final double aprilTagHeightMeters;
	private final double distanceFromAprilTagMeters;
	private final int aprilTagId;

	public AprilTagVisionData(
		VisionSource<? extends AprilTagVisionData> source,
		Pose3d estimatedRobotPosition,
		double timestamp,
		StandardDeviations3D standardDeviations,
		double aprilTagHeightMeters,
		double distanceFromAprilTagMeters,
		int aprilTagId
	) {
		super(source, estimatedRobotPosition, timestamp);
		this.standardDeviations = standardDeviations;
		this.aprilTagHeightMeters = aprilTagHeightMeters;
		this.distanceFromAprilTagMeters = distanceFromAprilTagMeters;
		this.aprilTagId = aprilTagId;
	}

	public StandardDeviations3D getStandardDeviations() {
		return standardDeviations;
	}

	public double getAprilTagHeightMeters() {
		return aprilTagHeightMeters;
	}

	public double getDistanceFromAprilTagMeters() {
		return distanceFromAprilTagMeters;
	}

	public int getTrackedAprilTagId() {
		return aprilTagId;
	}

}
