package frc.robot.vision.limelights;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

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

	public List<LimelightRawData> getAllAvailableLimelightData() {
		List<LimelightRawData> limelightsData = new ArrayList<>();

		for (Limelight limelight : limelights) {
			limelight.updateLimelight();
			Optional<Pair<Pose3d, Double>> observation = limelight.getUpdatedPose3DEstimation();

			if (observation.isPresent()) {
				LimelightRawData limelightRawData = new LimelightRawData(
					observation.get().getFirst(),
					limelight.getAprilTagHeight(),
					limelight.getDistanceFromAprilTag(),
					observation.get().getSecond()
				);
				limelightsData.add(limelightRawData);
			}
		}

		return limelightsData;
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
