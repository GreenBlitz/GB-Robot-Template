package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.constants.DirectoryPaths;
import frc.constants.field.Field;
import frc.constants.field.enums.FieldType;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.data.VisionData;
import frc.robot.vision.objectdetection.LimeLightObjectDetector;
import frc.robot.vision.sources.VisionSource;
import frc.robot.vision.sources.limelights.DynamicSwitchingLimelight;
import frc.utils.Filter;
import frc.utils.alerts.Alert;
import frc.utils.math.AngleUnit;

import java.io.IOException;
import java.util.List;

public class VisionConstants {

	public static final String FILTERED_DATA_LOGPATH_ADDITION = "FilteredData/";

	public static final String NON_FILTERED_DATA_LOGPATH_ADDITION = "NonFilteredData/";

	public static final String VISION_SOURCE_LOGPATH_ADDITION = "VisionSource/";

	public static final String OBJECT_DETECTOR_SOURCE_LOGPATH = "ObjectDetection/";

	public static final String MULTI_VISION_SOURCES_LOGPATH = "MultiVisionSources/";

	public static final String DYNAMIC_LIMELIGHT_MEGATAG1_SOURCE_NAME = "independentPoseEstimatingLimelight";

	public static final String DYNAMIC_LIMELIGHT_MEGATAG2_SOURCE_NAME = "headingRequiringLimelight";


	public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = getAprilTagFieldLayout(Field.FIELD_TYPE);

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

	private static AprilTagFieldLayout getAprilTagFieldLayout(FieldType fieldType) {
		return switch (fieldType) {
			case WELDED -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
			case ANDY_MARK -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
		};
	}

	public static final int LIMELIGHT_ENTRY_ARRAY_LENGTH = 6;

	public static final int NO_APRILTAG_ID = -1;

	public static Rotation2d ROLL_FILTER_TOLERANCE = Rotation2d.fromDegrees(5);

	public static Rotation2d PITCH_FILTER_TOLERANCE = Rotation2d.fromDegrees(5);

	public static Rotation2d YAW_FILTER_TOLERANCE = Rotation2d.fromDegrees(2);

	public static double ROBOT_POSITION_IN_FIELD_TOLERANCE_METERS = 0.1;

	public static double ROBOT_DISTANCE_TO_GROUND_TOLERANCE_METERS = 0.5;

	public static int[] TAGS_TO_IGNORE_FOR_LIMELIGHT_FEEDER = {4, 5, 14, 15};

	public static final Filter<VisionData> DEFAULT_VISION_FILTER = VisionFilters.isInField(ROBOT_POSITION_IN_FIELD_TOLERANCE_METERS)
		.and(VisionFilters.isRollAtAngle(Rotation2d.fromDegrees(0), ROLL_FILTER_TOLERANCE))
		.and(VisionFilters.isPitchAtAngle(Rotation2d.fromDegrees(0), PITCH_FILTER_TOLERANCE))
		.and(VisionFilters.isOnGround(ROBOT_DISTANCE_TO_GROUND_TOLERANCE_METERS));

	public static final Pose3d LIMELIGHT_LEFT_CAMERA_ROBOT_POSE = new Pose3d(
		new Translation3d(0.215, -0.11, 0.508),
		AngleUnit.DEGREES.toRotation3d(-8.06180374425555, -27.07784559039065, -22.52372569716833)
	);

	public static final Pose3d LIMELIGHT_RIGHT_CAMERA_ROBOT_POSE = new Pose3d(
		new Translation3d(0.2022, 0.13, 0.508),
		AngleUnit.DEGREES.toRotation3d(10.612258493096334, -27.18966371065684, 20.10328620400214)
	);

	public static final Pose3d OBJECT_DETECTION_LIMELIGHT_CAMERA_POSE = new Pose3d(
		new Translation3d(-0.08, 0.23, 0.865),
		AngleUnit.DEGREES.toRotation3d(0, -27, 180)
	);

	public static final String NAME_ENTRY_NO_OBJECT_VALUE = "none";

	public static final double ALGAE_HEIGHT_TO_WIDTH_RATIO_TOLERANCE = 0.5;

	public static final double ALGAE_HEIGHT_TO_WIDTH_RATIO = 1;

	public static final int OBJECT_CELL_AMOUNT_IN_RAW_DETECTIONS_ENTRY = 12;

	public static final Rotation2d LIMELIGHT_3_HORIZONTAL_FOV = Rotation2d.fromDegrees(62.5);

	public static final Rotation2d LIMELIGHT_3_VERTICAL_FOV = Rotation2d.fromDegrees(48.9);

	public static final Translation2d LIMELIGHT_OBJECT_RESOLUTION_PIXELS = new Translation2d(100, 100);

	public static final int EDGE_PIXEL_TOLERANCE = 10;

	public static double MAX_VALID_ALGAE_DISTANCE_METERS = 3;

	public static final VisionSource<AprilTagVisionData> LIMELIGHT_LEFT = new DynamicSwitchingLimelight(
		true,
		"limelight-left",
		VisionConstants.MULTI_VISION_SOURCES_LOGPATH,
		"limelight4-front",
		VisionConstants.DEFAULT_VISION_FILTER,
		LIMELIGHT_LEFT_CAMERA_ROBOT_POSE
	);

	public static final VisionSource<AprilTagVisionData> LIMELIGHT_RIGHT = new DynamicSwitchingLimelight(
		true,
		"limelight",
		VisionConstants.MULTI_VISION_SOURCES_LOGPATH,
		"limelight3gb-front",
		VisionConstants.DEFAULT_VISION_FILTER,
		LIMELIGHT_RIGHT_CAMERA_ROBOT_POSE
	);

	public static final LimeLightObjectDetector LIMELIGHT_OBJECT = new LimeLightObjectDetector(
		VisionConstants.OBJECT_DETECTOR_SOURCE_LOGPATH,
		"limelight-object",
		OBJECT_DETECTION_LIMELIGHT_CAMERA_POSE
	);

	public static final List<VisionSource<AprilTagVisionData>> VISION_SOURCES = List.of(LIMELIGHT_LEFT, LIMELIGHT_RIGHT);

}
