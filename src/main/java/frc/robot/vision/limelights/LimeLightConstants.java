package frc.robot.vision.limelights;

import edu.wpi.first.math.geometry.Rotation2d;

public class LimeLightConstants {

	public static final String ESTIMATION_LOGPATH_PREFIX = "Estimation/";

	public static final int LIMELIGHT_ENTRY_ARRAY_LENGTH = 7;

	public static final int NO_APRILTAG_ID = -1;

	public final static double APRIL_TAG_HEIGHT_TOLERANCE_METERS = 0.07;

	public static final double POSITION_NORM_TOLERANCE = 0.1;

	public static final double ROTATION_NORM_TOLERANCE = 0.2;

	public static final Rotation2d ROLL_TOLERANCE = Rotation2d.fromDegrees(10);

	public static final Rotation2d PITCH_TOLERANCE = Rotation2d.fromDegrees(10);

	public static final double ROBOT_TO_GROUND_TOLERANCE_METERS = 0.2;

	public final static LimelightFiltersTolerances DEFAULT_LIMELIGHT_FILTERS_TOLERANCES = new LimelightFiltersTolerances(
		APRIL_TAG_HEIGHT_TOLERANCE_METERS,
		POSITION_NORM_TOLERANCE,
		ROTATION_NORM_TOLERANCE,
		ROLL_TOLERANCE,
		PITCH_TOLERANCE,
		ROBOT_TO_GROUND_TOLERANCE_METERS
	);

	public static final double VISION_STANDARD_DEVIATION_ANGLES = 0.6;

	public static final double TIME_TO_FIX_POSE_ESTIMATION_SECONDS = 1;

}
