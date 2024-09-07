package frc.robot.poseestimator.limelights;

import edu.wpi.first.math.geometry.Rotation2d;

public class VisionConstants {

	public static final String[] LIMELIGHT_NAMES = new String[] {"limelight-front", "limelight-back", "limelight-gb"};

	public static final SmartLimelightsConfig DEFAULT_CONFIG = new SmartLimelightsConfig(
		"SmartLimelights/",
		"LimelightsHardware/",
		Rotation2d.fromDegrees(20), // ! shall be calibrated
		0.2 // ! shall be calibrated
	);

	public static final String LIMELIGHT_LOGPATH_PREFIX = "Limelight ";

	public static final String ESTIMATION_LOGPATH_PREFIX = "estimation_";


	public static int LIMELIGHT_ENTRY_ARRAY_LENGTH = 7;

	public final static double VISION_TO_STANDARD_DEVIATION = 10;

	public final static double APRIL_TAG_HEIGHT_METERS = 1.2397;

	public final static double APRIL_TAG_HEIGHT_TOLERANCE_METERS = 0.07;


}
