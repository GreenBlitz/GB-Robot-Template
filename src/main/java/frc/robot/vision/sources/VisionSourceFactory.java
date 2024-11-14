package frc.robot.vision.sources;

import frc.robot.RobotManager;
import frc.robot.vision.CameraType;
import frc.robot.vision.RawVisionData;
import frc.robot.vision.VisionConstants;

public class VisionSourceFactory {

	public static VisionSource<RawVisionData> createPoseEstimatingSource(CameraType type, String sourceName) {
		if (RobotManager.isReal()) {
			return switch (type) {
				case LIMELIGHT -> new LimeLightSource(sourceName, VisionConstants.SOURCE_LOGPATH);
				case PHOTON_VISION -> null;
			};
		}
		return null;
	}

}
