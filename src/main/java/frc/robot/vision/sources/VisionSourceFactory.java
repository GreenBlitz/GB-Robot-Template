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
			;
			return switch (type) {
				case LIMELIGHT ->
					new SimulatedSource(sourceName, simulatedLocation.get(), VisionConstants.LIMELIGHT_3_SIMULATED_SOURCE_CONFIGURATION);
				case PHOTON_VISION -> null;
			};
		}
		return null;
	}

}
