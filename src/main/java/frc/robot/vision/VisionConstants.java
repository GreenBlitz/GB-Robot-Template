package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.vision.sources.simulationsource.SimulatedSourceConfiguration;

public class VisionConstants {

	public static final String FILTERED_ESTIMATION_LOGPATH_ADDITION = "FilteredEstimation/";

	public static final String NON_FILTERED_ESTIMATION_LOGPATH_ADDITION = "NonFilteredEstimation/";

	public static final String FILTERED_OUT_RAW_DATA_LOGPATH_ADDITION = "FilteredOutRawData/";

	public static final String SOURCE_LOGPATH_ADDITION = "VisionSource/";


	public static final int dLIMELIGHT_ENTRY_ARRAY_LENGTH  = 6;

	public static final int dNO_APRILTAG_ID  = -1;

	public static final Rotation2d ROLL_TOLERANCE = Rotation2d.fromDegrees(10);

	public static final Rotation2d PITCH_TOLERANCE = Rotation2d.fromDegrees(10);

	public static final double ROBOT_DISTANCE_FROM_GROUND_TOLERANCE_METERS = 0.2;

	public static final VisionFiltersTolerances DEFAULT_VISION_FILTERS_TOLERANCES = new VisionFiltersTolerances(
		ROLL_TOLERANCE,
		PITCH_TOLERANCE,
		ROBOT_DISTANCE_FROM_GROUND_TOLERANCE_METERS
	);

	public static final VisionFiltererConfig DEFAULT_VISION_FILTERER_CONFIG = new VisionFiltererConfig(
		SOURCE_LOGPATH_ADDITION,
		DEFAULT_VISION_FILTERS_TOLERANCES,
		VisionFilters::doesRawDataPassAllFilters
	);

	public static SimulatedSourceConfiguration LIMELIGHT_3_SIMULATED_SOURCE_CONFIGURATION = new SimulatedSourceConfiguration(
		0.1,
		0.1,
		0.05,
		2.5,
		3,
		Rotation2d.fromDegrees(61)
	);

	public static final double VISION_ANGLE_STANDARD_DEVIATION = 0.6;

}

