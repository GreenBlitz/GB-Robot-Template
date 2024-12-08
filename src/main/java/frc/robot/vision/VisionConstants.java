package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;

public class VisionConstants {

	public static final String FILTERED_ESTIMATION_LOGPATH_ADDITION = "FilteredEstimation/";

	public static final String NON_FILTERED_ESTIMATION_LOGPATH_ADDITION = "NonFilteredEstimation/";

	public static final String SOURCE_LOGPATH_ADDITION = "VisionSource/";

	public static final Rotation2d ROLL_TOLERANCE = Rotation2d.fromDegrees(10);

	public static final Rotation2d PITCH_TOLERANCE = Rotation2d.fromDegrees(10);

	public static final double ROBOT_DISTANCE_FROM_GROUND_TOLERANCE_METERS = 0.2;

}
