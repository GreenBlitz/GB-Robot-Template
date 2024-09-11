package frc.robot.poseestimator.limelights;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class MultiLimelightsRawData {

	private List<Limelight> limelights;
	private String logPath;

	public MultiLimelightsRawData(String[] names, String hardwareLogPath) {
		this.logPath = hardwareLogPath;
		this.limelights = new ArrayList<>();

		for (String limelightName : names) {
			limelights.add(new Limelight(limelightName, hardwareLogPath));
		}
	}

	public List<LimelightRawData> getAllAvailableLimelightData() {
		List<LimelightRawData> limelightsData = new ArrayList<>();

		for (Limelight limelight : limelights) {
			Logger.recordOutput("ll/" + limelight.getName(), true);
			Optional<Pair<Pose2d, Double>> observation = limelight.getUpdatedPose2DEstimation();
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

}
