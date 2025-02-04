package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmConstants {

	public static final Rotation2d FORWARD_SOFTWARE_LIMIT = Rotation2d.fromDegrees(250);
	public static final Rotation2d REVERSED_SOFTWARE_LIMIT = Rotation2d.fromDegrees(-50);
	public static final double LENGTH_METERS = 1;
	public static final double MASS_KG = 5;

	public static final Rotation2d CRUISE_VELOCITY_ANGLES_PER_SECOND = Rotation2d.fromRotations(10);
	public static final Rotation2d ACCELERATION_ANGLES_PER_SECOND_SQUARED = Rotation2d.fromRotations(512);

	public static final double CALIBRATION_MAX_POWER = 0.2;

}
