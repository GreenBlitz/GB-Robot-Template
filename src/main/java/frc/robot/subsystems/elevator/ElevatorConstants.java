package frc.robot.subsystems.elevator;

import frc.constants.MathConstants;

public class ElevatorConstants {

	public static final double DRUM_DIAMETER_METERS = 0.04138 / Math.PI;
	public static final double DRUM_RADIUS_METERS = DRUM_DIAMETER_METERS / 2;
	public static final double MASS_KG = 5;
	public static final double MINIMUM_HEIGHT_METERS = 0;
	public static final double MAXIMUM_HEIGHT_METERS = 1.4;
	public static final double FIRST_STAGE_MAXIMUM_HEIGHT_METERS = 0.7;

	public final static double GEAR_RATIO = 17.0 / 63.0;

	public static final double REVERSE_SOFT_LIMIT_VALUE_METERS = 0.01;

	public static final double FORWARD_SOFT_LIMIT_VALUE_METERS = 1.274;

	public static final double CRUISE_VELOCITY_METERS_PER_SECOND = 10;
	public static final double ACCELERATION_METERS_PER_SECOND_SQUARED = 512;

}
