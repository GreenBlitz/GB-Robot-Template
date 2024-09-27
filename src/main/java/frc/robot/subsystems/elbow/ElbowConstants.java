package frc.robot.subsystems.elbow;

import edu.wpi.first.math.geometry.Rotation2d;

public class ElbowConstants {

	public static final String LOG_PATH = "Elbow/";

	public static final Rotation2d FORWARD_LIMIT = Rotation2d.fromDegrees(90);
	public static final Rotation2d BACKWARD_LIMIT = Rotation2d.fromDegrees(-78);

	public static final double GEAR_RATIO = 1.0 / (28.0 * (60.0 / 16.0));

	protected static final Rotation2d MINIMUM_ACHIEVABLE_POSITION = Rotation2d.fromDegrees(-82);

}
