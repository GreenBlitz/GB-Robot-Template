package frc.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.vision.data.VisionData;
import frc.utils.Filter;
import frc.utils.alerts.Alert;

import java.io.IOException;

public class VisionConstants {

	public static final String FILTERED_DATA_LOGPATH_ADDITION = "FilteredData/";

	public static final String NON_FILTERED_DATA_LOGPATH_ADDITION = "NonFilteredData/";

	public static final String VISION_SOURCE_LOGPATH_ADDITION = "VisionSource/";

	public static final String MULTI_VISION_SOURCES_LOGPATH = "MultiVisionSources/";

	public static final String DYNAMIC_LIMELIGHT_MEGATAG1_SOURCE_NAME = "independentPoseEstimatingLimelight";

	public static final String DYNAMIC_LIMELIGHT_MEGATAG2_SOURCE_NAME = "headingRequiringLimelight";

	public static double DEFAULT_RATIO_BETWEEN_IMU_AND_SOURCE_LIMELIGHT_4 = 0.001;

	public static double LIMELIGHT_4_MAXIMUM_LIMELIGHT_TEMPERATURE_CELSIUS = 80;

	public static double LIMELIGHT_4_MAXIMUM_CPU_TEMPERATURE_CELSIUS = 80;

	public static boolean REGULATE_TEMPERATURE_IN_LIMELIGHT_4 = false;

	public static int FALLBACK_SKIPPED_FRAMES_LIMELIGHT_4 = 100;

	// solves heating issues on limelight 4. According to the official docs, 50-100 should resolve any heating issues.
	public static int DEFAULT_SKIPPED_FRAMES_LIMELIGHT_4 = 2;


	public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = getAprilTagFieldLayout();

	private static AprilTagFieldLayout getAprilTagFieldLayout() {
		try {
			return AprilTagFieldLayout.loadFromResource(DirectoryPaths.APRIL_TAG_FIELD_CONFIG_FILE_PATH.toString());
		} catch (IOException ioException) {
			new Alert(
				Alert.AlertType.WARNING,
				"Cannot read april tag field layout from " + DirectoryPaths.APRIL_TAG_FIELD_CONFIG_FILE_PATH + ", using default field layout"
			).report();
			return AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
		}
	}

	public static final int LIMELIGHT_ENTRY_ARRAY_LENGTH = 6;

	public static final int NO_APRILTAG_ID = -1;

	public static final Filter<VisionData> DEFAULT_VISION_FILTER = Filter.nonFilteringFilter();

}
