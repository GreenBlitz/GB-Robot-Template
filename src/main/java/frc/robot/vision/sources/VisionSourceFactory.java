package frc.robot.vision.sources;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotManager;
import frc.robot.vision.CameraType;
import frc.robot.vision.RawVisionData;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.sources.simulationsource.SimulatedSource;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class VisionSourceFactory {

	public static VisionSource<
		RawVisionData> createPoseEstimatingSource(CameraType type, String sourceName, Optional<Supplier<Pose2d>> simulatedLocation) {
		if (RobotManager.isReal()) {
			return switch (type) {
				case LIMELIGHT -> new LimeLightSource(sourceName, VisionConstants.SOURCE_LOGPATH);
				case PHOTON_VISION -> null;
			};
		} else if (RobotManager.isSimulation()) {
			if (simulatedLocation.isEmpty()) {
				AlertManager.addAlert(new PeriodicAlert(Alert.AlertType.WARNING, "can'tAccessRobotSimulatedLocation", () -> true));
				return null;
			}

			return switch (type) {
				case LIMELIGHT ->
					new SimulatedSource(sourceName, simulatedLocation.get(), VisionConstants.LIMELIGHT_3_SIMULATED_SOURCE_CONFIGURATION);
				case PHOTON_VISION -> null;
			};
		}
		return null;
	}

	public static List<VisionSource<RawVisionData>> generateDefaultSources(Supplier<Pose2d> simulatedLocation) {
		return new ArrayList<>(
			List.of(
				VisionSourceFactory.createPoseEstimatingSource(CameraType.LIMELIGHT, "limelight-front", Optional.of(simulatedLocation)),
				VisionSourceFactory.createPoseEstimatingSource(CameraType.LIMELIGHT, "limelight-back", Optional.of(simulatedLocation))
			)
		);
	}

}
