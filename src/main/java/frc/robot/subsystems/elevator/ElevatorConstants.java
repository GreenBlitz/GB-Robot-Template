package frc.robot.subsystems.elevator;


public class ElevatorConstants {

	public static final double DRUM_DIAMETER_METERS = 0.04138;
	public static final double DRUM_RADIUS_METERS = DRUM_DIAMETER_METERS / 2;
	public static final double MASS_KG = 9.5154;
	public static final double MINIMUM_HEIGHT_METERS = 0;
	public static final double MAXIMUM_HEIGHT_METERS = 1.17;
	public static final double FIRST_STAGE_MAXIMUM_HEIGHT_METERS = 0.7;

	public final static double GEAR_RATIO = (63.0 / 17.0);

	public static final double REVERSE_SOFT_LIMIT_VALUE_METERS = 0;

	public static final double FORWARD_SOFT_LIMIT_VALUE_METERS = 1.13;

	public static final double CRUISE_VELOCITY_METERS_PER_SECOND = 1;
	public static final double ACCELERATION_METERS_PER_SECOND_SQUARED = 2;

}
