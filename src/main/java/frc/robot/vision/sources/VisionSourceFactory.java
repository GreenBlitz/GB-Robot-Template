package frc.robot.vision.sources;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.RobotManager;
import frc.robot.vision.CameraType;
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

	public static RobotPoseEstimatingVisionSource createPoseEstimatingSource(
		CameraType type,
		String sourceName,
		Optional<Supplier<Pose2d>> simulatedLocation,
		Optional<Pose3d> simulatedCameraRelativeToRobot
	) {
		if (RobotManager.isReal()) {
			return switch (type) {
				case LIMELIGHT -> new LimeLightSource(sourceName, VisionConstants.SOURCE_LOGPATH_ADDITION);
				case PHOTON_VISION -> null;
			};
		} else {
			if (simulatedLocation.isEmpty() || simulatedCameraRelativeToRobot.isEmpty()) {
				AlertManager.addAlert(new PeriodicAlert(Alert.AlertType.WARNING, "can'tAccessRobotSimulatedLocation", () -> true));
				return null;
			}

			return switch (type) {
				case LIMELIGHT ->
					new SimulatedSource(
						sourceName,
						simulatedLocation.get(),
						simulatedCameraRelativeToRobot.get(),
						VisionConstants.LIMELIGHT_3_SIMULATED_SOURCE_CONFIGURATION
					);
				case PHOTON_VISION -> null;
			};
		}
	}

	public static List<RobotPoseEstimatingVisionSource> generateDefaultPoseEstimatingSources(Supplier<Pose2d> simulatedLocation) {
		return new ArrayList<>(
			List.of(
				VisionSourceFactory.createPoseEstimatingSource(
					CameraType.LIMELIGHT,
					"limelight-front",
					Optional.of(simulatedLocation),
					Optional.of(new Pose3d())
				),
				VisionSourceFactory.createPoseEstimatingSource(
					CameraType.LIMELIGHT,
					"limelight-back",
					Optional.of(simulatedLocation),
					Optional.of(new Pose3d())
				)
			)
		);
	}

}
