package frc.robot.vision;

import frc.constants.VisionConstants;

public class VisionUtils {

	public static double getAprilTagHeightByID(int id) {
		return VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTags().get(id).pose.getZ();
	}

}
