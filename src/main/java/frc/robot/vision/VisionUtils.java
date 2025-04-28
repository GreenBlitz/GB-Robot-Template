package frc.robot.vision;

public class VisionUtils {

	public static double getAprilTagHeightByID(int id) {
		return VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTags().get(id).pose.getZ();
	}

}
