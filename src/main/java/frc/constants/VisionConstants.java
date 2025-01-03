package frc.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.vision.VisionFilters;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.sources.VisionSource;
import frc.robot.vision.sources.limelights.LimeLightSource;
import frc.utils.Filter;
import frc.utils.alerts.Alert;

import java.io.IOException;
import java.util.List;

public class VisionConstants {

	public static final String FILTERED_DATA_LOGPATH_ADDITION = "FilteredData/";

	public static final String NON_FILTERED_DATA_LOGPATH_ADDITION = "NonFilteredData/";

	public static final String SOURCE_LOGPATH_ADDITION = "VisionSource/";

	public static final String MULTI_VISION_SOURCES_LOGPATH = "MultiVisionSources/";


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

	public static final int LATENCY_BOTPOSE_INDEX = 6;

	public static final boolean REQUIRE_HEADING_TO_ESTIMATE_ANGLE = false;


	public static final Filter<AprilTagVisionData> DEFAULT_VISION_FILTER = VisionFilters
		.extractFilterToPreformPolymorphism(VisionFilters.isOnGround(0.2)); // .and(VisionFilters.isAprilTagHeightInTolerance(0.5, 1.2));

	public static List<VisionSource<AprilTagVisionData>> DEFAULT_VISION_SOURCES = List.of(
		new LimeLightSource("limelight-front", MULTI_VISION_SOURCES_LOGPATH, DEFAULT_VISION_FILTER),
		new LimeLightSource("limelight-back", MULTI_VISION_SOURCES_LOGPATH, DEFAULT_VISION_FILTER)
	);

}