package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.utils.alerts.Alert;

import java.io.IOException;

public class VisionConstants {

	public static final String FILTERED_DATA_LOGPATH_ADDITION = "FilteredData/";

	public static final String NON_FILTERED_DATA_LOGPATH_ADDITION = "NonFilteredData/";

	public static final String SOURCE_LOGPATH_ADDITION = "VisionSource/";


	public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = getAprilTagFieldLayout();

	private static AprilTagFieldLayout getAprilTagFieldLayout() {
		try {
<<<<<<< HEAD
			return AprilTagFieldLayout.loadFromResource(APRIL_TAG_FIELD_CONFIG_FILE_PATH.toString());
		} catch (IOException ioException) {
||||||| parent of 029024848 (refactor(vision): improve variable naming, code reusing, and use constants without direct import)
			return AprilTagFieldLayout.loadFromResource(APRIL_TAG_FIELD_CONFIG_FILE_PATH.toString());
		} catch (IOException exception) {
=======
			return AprilTagFieldLayout.loadFromResource(DirectoryPaths.APRIL_TAG_FIELD_CONFIG_FILE_PATH.toString());
		} catch (IOException exception) {
>>>>>>> 029024848 (refactor(vision): improve variable naming, code reusing, and use constants without direct import)
			new Alert(
				Alert.AlertType.WARNING,
				"Cannot read april tag field layout from " + DirectoryPaths.APRIL_TAG_FIELD_CONFIG_FILE_PATH + ", using default field layout"
			).report();
			return AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
		}
	}

}
