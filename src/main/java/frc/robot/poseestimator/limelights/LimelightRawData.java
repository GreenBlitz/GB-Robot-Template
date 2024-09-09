package frc.robot.poseestimator.limelights;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class LimelightRawData extends GBSubsystem {

	private List<Limelight> limelights;


	public LimelightRawData(String[] names, String hardwareLogPath) {
		super(hardwareLogPath);

		this.limelights = new ArrayList<>();
		for (String limelightName : names) {
			Logger.recordOutput(limelightName + "Created");
			System.out.println(limelightName + " created");
			limelights.add(new Limelight(limelightName, hardwareLogPath));
		}
	}

	public List<LimelightData> getAllAvlilableLimelightData() {
		List<LimelightData> limelightsData = List.of();

		for (Limelight limelight : limelights) {
			Optional<Pair<Pose2d, Double>> observation = limelight.getUpdatedPose2DEstimation();
			if (observation.isPresent()) {
				LimelightData limelightData = new LimelightData(
					observation.get().getFirst(),
					limelight.getAprilTagHeight(),
					limelight.getDistanceFromAprilTag(),
					observation.get().getSecond()
				);
				limelightsData.add(limelightData);
			}
		}

		return limelightsData;
	}

	@Override
	protected void subsystemPeriodic() {}


}
