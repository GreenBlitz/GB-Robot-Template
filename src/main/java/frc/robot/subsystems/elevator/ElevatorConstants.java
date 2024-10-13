package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;

public class ElevatorConstants {

	public static final String LOG_PATH = "Subsystems/Elevator/";

	public final static double GEAR_RATIO = 48.0 / 14.0;

	public static final double MINIMUM_ACHIEVABLE_POSITION_METERS = 0;

	public static final double MOTOR_ROTATIONS_TO_METERS_CONVERSION_RATIO = 1;

	public static final Rotation2d REVERSE_SOFT_LIMIT_VALUE = Rotation2d.fromRotations(0);

	public static final Rotation2d FORWARD_SOFT_LIMIT_VALUE = Rotation2d.fromRotations(0);

}
