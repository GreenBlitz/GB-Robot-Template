package frc.robot.subsystems.elbow;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class ElbowConstants {

	public static final String LOG_PATH = "Subsystems/Elbow/";

	public static final Rotation2d FORWARD_LIMIT = Rotation2d.fromDegrees(90);
	public static final Rotation2d BACKWARD_LIMIT = Rotation2d.fromDegrees(-78);

	public static final double GEAR_RATIO = 1.0 / (28.0 * (60.0 / 16.0));

	protected static final Rotation2d MINIMUM_ACHIEVABLE_POSITION = Rotation2d.fromDegrees(-82);

	public static final Translation3d ELBOW_POSITION_RELATIVE_TO_ROBOT = new Translation3d(-0.1, 0, 0.6);

}
