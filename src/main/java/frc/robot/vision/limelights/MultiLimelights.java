package frc.robot.vision.limelights;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.vision.VisionRawData;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class MultiLimelights {

	private final List<Limelight> limelights;

	public MultiLimelights(String[] names, String hardwareLogPath) {
		this.limelights = new ArrayList<>();

		for (String limelightName : names) {
			limelights.add(new Limelight(limelightName, hardwareLogPath));
		}
	}

	public void updateGyroAngles(GyroAngleValues gyroAngleValues) {
		for (Limelight limelight : limelights) {
			limelight.updateGyroAngleValues(gyroAngleValues);
		}
	}

	public List<VisionRawData> getAllAvailableLimelightData() {
		List<VisionRawData> limelightsData = new ArrayList<>();

		for (Limelight limelight : limelights) {
			limelight.updateLimelight();
			Optional<Pair<Pose3d, Double>> observation = limelight.getUpdatedPose3DEstimation();

			if (observation.isPresent()) {
				double timestamp = observation.get().getSecond();
				VisionRawData visionData = new VisionRawData(
					limelight.getCameraName(),
					observation.get().getFirst(),
					calculateAmbiguity(limelight.getAprilTagHeight()),
					timestamp,
					limelight.getLatency()
				);
				limelightsData.add(visionData);
			}
		}

		return limelightsData;
	}

	// we should find a better function later
	private double calculateAmbiguity(double aprilTagHeightMeters) {
		double heightPower3 = Math.pow(aprilTagHeightMeters, 3);
		return heightPower3 / (heightPower3 + 200);
	}

	public List<Rotation2d> getAllRobotHeadingEstimations() {
		List<Rotation2d> headingEstimations = new ArrayList<>();
		for (Limelight limelight : limelights) {
			Optional<Rotation2d> heading = limelight.getRobotHeading();
			heading.ifPresent(headingEstimations::add);
		}
		return headingEstimations;
	}

}
