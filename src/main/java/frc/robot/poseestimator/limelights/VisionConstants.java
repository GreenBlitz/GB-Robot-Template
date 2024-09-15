package frc.robot.poseestimator.limelights;

import edu.wpi.first.math.geometry.Rotation2d;

public class VisionConstants {

	public static final VisionObservationFilteredConfig DEFAULT_CONFIG = new VisionObservationFilteredConfig(
		"VisionObservationFiltered/",
		"LimelightsHardware/",
		Rotation2d.fromDegrees(20), // ! shall be calibrated
		0.2, // ! shall be calibrated
		new String[] {"limelight-front", "limelight-back"}
	);

	public static final String ESTIMATION_LOGPATH_PREFIX = "Estimation";

	public static final int LIMELIGHT_ENTRY_ARRAY_LENGTH = 7;

	public final static double APRIL_TAG_DISTANCE_TO_STANDARD_DEVIATIONS_FACTOR = 10;

	public final static double APRIL_TAG_HEIGHT_TOLERANCE_METERS = 0.07;

	public static final double STANDARD_DEVIATION_VISION_ANGLE = 0.6;

}
