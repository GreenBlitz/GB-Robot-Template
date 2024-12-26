package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.utils.alerts.Alert;

import java.io.IOException;

public class VisionConstants {

	public static final String FILTERED_DATA_LOGPATH_ADDITION = "FilteredData/";

	public static final String NON_FILTERED_DATA_LOGPATH_ADDITION = "NonFilteredData/";

	public static final String SOURCE_LOGPATH_ADDITION = "VisionSource/";


	private static final String APRIL_TAG_FIELD_CONFIG_FILE_PATH = "april_tag_field_config.json";

	public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = getAprilTagFieldLayout();

	private static AprilTagFieldLayout getAprilTagFieldLayout() {
		try {
			return AprilTagFieldLayout.loadFromResource(APRIL_TAG_FIELD_CONFIG_FILE_PATH);
		} catch (IOException exception) {
			new Alert(
				Alert.AlertType.WARNING,
				"Cannot read april tag field layout from " + APRIL_TAG_FIELD_CONFIG_FILE_PATH + ", using default field layout"
			).report();
			return AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
		}
	}

	public static final int LIMELIGHT_ENTRY_ARRAY_LENGTH = 6;

	public static final int NO_APRILTAG_ID = -1;

}
