package frc.robot.subsystems.constants.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {

	public final static double kP = 12;
	public final static double kI = 2;
	public final static double kD = 0;
	public final static double kV = 0.0;
	public final static double kS = 0.0;
	public final static double kA = 0.0;
	public final static double kP_SIM = 0.4;
	public final static double kI_SIM = 0;
	public final static double kD_SIM = 0;
	public final static double kS_SIM = 0.0;
    public final static double kV_SIM = 0.1385;
    public final static double kA_SIM = 0;
    public final static double SENSOR_TO_MECHANISM_RATIO_MASTER = 40.0 / 36.0;
    public final static double SENSOR_TO_MECHANISM_RATIO_FOLLOWER = 40.0 / 36.0;
    public final static double MOMENT_OF_INERTIA = 0.01;
    public final static int CURRENT_LIMIT = 40;
    public final static double WHEEL_RADIUS_METERS = 0.05;
	public final static Rotation2d FLYWHEEL_VELOCITY_TOLERANCE_RPS = Rotation2d.fromDegrees(5);

}
