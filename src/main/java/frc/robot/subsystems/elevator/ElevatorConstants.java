package frc.robot.subsystems.elevator;

import frc.robot.constants.MathConstants;

public class ElevatorConstants {
	
	public static final String LOG_PATH = "Subsystems/Elevator/";
	
	public final static double GEAR_RATIO = 14.0 / 48;
	
	public static final double MINIMUM_ACHIEVABLE_POSITION_METERS = 0;
	
	public static final double MOTOR_ROTATIONS_TO_METERS_CONVERSION_RATIO = 0.47 / (0.34 * MathConstants.HALF_CIRCLE.getRadians());
	
	public static final double REVERSE_SOFT_LIMIT_VALUE_METERS = 0.02;
	
	public static final double FORWARD_SOFT_LIMIT_VALUE_METERS = 0.24;
	
}
