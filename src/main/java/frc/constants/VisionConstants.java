package frc.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.vision.VisionFilters;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.sources.VisionSource;
import frc.robot.vision.sources.limelights.DynamicSwitchingLimelight;
import frc.robot.vision.sources.limelights.LimelightFactory;
import frc.utils.Filter;
import frc.robot.vision.data.AprilTagVisionData;
import frc.utils.Filter;
import frc.utils.alerts.Alert;

import java.io.IOException;
import java.util.List;
import java.util.function.Function;
import java.util.List;

public class VisionConstants {

	public static final String FILTERED_DATA_LOGPATH_ADDITION = "FilteredData/";

	public static final String NON_FILTERED_DATA_LOGPATH_ADDITION = "NonFilteredData/";

	public static final String VISION_SOURCE_LOGPATH_ADDITION = "VisionSource/";

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

	public static final boolean REQUIRE_HEADING_TO_ESTIMATE_ANGLE_DEFAULT_VALUE = true;

	public static final boolean REQUIRE_HEADING_TO_ESTIMATE_ANGLE = true;

	public static <T> Function<T, Boolean> CreateTrueFunction() {
		return (T iDontCare) -> true;
	}

	public static final Filter<AprilTagVisionData> DEFAULT_VISION_FILTER = Filter.nonFilteringFilter();
//		.polymorphAs(); // .and(VisionFilters.isAprilTagHeightInTolerance(0.5, 1.2));

	public static final List<VisionSource<AprilTagVisionData>> DEFAULT_VISION_POSEESTIMATING_SOURCES = List.of(
		new DynamicSwitchingLimelight(true, "limelight-back", MULTI_VISION_SOURCES_LOGPATH, VisionConstants.DEFAULT_VISION_FILTER)
//		LimelightFactory.createRobotHeadingEstimatingLimelight("limelight-back", MULTI_VISION_SOURCES_LOGPATH, VisionConstants.DEFAULT_VISION_FILTER)
//		new LimeLightSource("limelight-back", MULTI_VISION_SOURCES_LOGPATH, new Filter<>(data -> true))
	);

	public static final List<VisionSource<AprilTagVisionData>> DEFAULT_VISION_POSEESTIMATING_SOURCES2 = List.of(
		new DynamicSwitchingLimelight(true, "limelight-back", MULTI_VISION_SOURCES_LOGPATH, VisionFilters.isPitchAtAngle(
			Rotation2d.fromDegrees(0),
			Rotation2d.fromDegrees(1)
		).and(VisionFilters.isPitchAtAngle(
			Rotation2d.fromDegrees(0),
			Rotation2d.fromDegrees(1)
		)).polymorphAs())
	);

	public static final List<VisionSource<AprilTagVisionData>> DEFAULT_VISION_POSEESTIMATING_SOURCES3 = List.of(
		new DynamicSwitchingLimelight(true, "limelight-back", MULTI_VISION_SOURCES_LOGPATH, VisionFilters.isOnGround(0.06).polymorphAs())
	);

	public static final double VISION_STDEVS_FACTOR = 0.1;

}
