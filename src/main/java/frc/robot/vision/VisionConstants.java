package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;

public class VisionConstants {

	public static final String ESTIMATION_LOGPATH_PREFIX = "FilteredEstimation/";

	public static final String NON_FILTERED_ESTIMATION_LOGPATH_PREFIX = "NonFilteredEstimation/";

	public static final String FILTERED_OUT_RAW_DATA = "FilteredOutRawData/";

	public static final String SOURCE_LOGPATH = "VisionSource/";

	public static final int LIMELIGHT_ENTRY_ARRAY_LENGTH = 7;

	public static final int NO_APRILTAG_ID = -1;

	public static final Rotation2d ROLL_TOLERANCE = Rotation2d.fromDegrees(10);

	public static final Rotation2d PITCH_TOLERANCE = Rotation2d.fromDegrees(10);

	public static final double ROBOT_TO_GROUND_TOLERANCE_METERS = 0.2;

	public static final VisionFiltersTolerances DEFAULT_VISION_FILTERS_TOLERANCES = new VisionFiltersTolerances(
		ROLL_TOLERANCE,
		PITCH_TOLERANCE,
		ROBOT_TO_GROUND_TOLERANCE_METERS
	);

	public static final double VISION_ANGLE_STANDARD_DEVIATION = 0.6;

}
