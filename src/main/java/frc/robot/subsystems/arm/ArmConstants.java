package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmConstants {

	public static final Rotation2d FORWARD_SOFTWARE_LIMIT = Rotation2d.fromDegrees(210);
	public static final Rotation2d MAXIMUM_POSITION = Rotation2d.fromDegrees(220);
	public static final Rotation2d ELEVATOR_CLOSED_REVERSED_SOFTWARE_LIMIT = Rotation2d.fromDegrees(-30);
	public static final Rotation2d ELEVATOR_OPEN_REVERSED_SOFTWARE_LIMIT = Rotation2d.fromDegrees(-50);
	public static final double ELEVATOR_HEIGHT_METERS_TO_CHANGE_SOFT_LIMIT = 0.3;
	public static final double LENGTH_METERS = 0.3;
	public static final double MASS_KG = 5;

	public static final Rotation2d CRUISE_VELOCITY_ANGLES_PER_SECOND = Rotation2d.fromRotations(4);
	public static final Rotation2d ACCELERATION_ANGLES_PER_SECOND_SQUARED = Rotation2d.fromRotations(4);

	public static final double CALIBRATION_MAX_POWER = 0.2;

}
