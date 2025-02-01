package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmConstants {

	public static final Rotation2d FORWARD_SOFTWARE_LIMIT = Rotation2d.fromDegrees(250);
	public static final Rotation2d REVERSED_SOFTWARE_LIMIT = Rotation2d.fromDegrees(-50);
	public static final double LENGTH_METERS = 1;
	public static final double MASS_KG = 5;

	public static final double CRUISE_VELOCITY_ROTATIONS_PER_SECOND = 10;
	public static final double ACCELERATION_ROTATIONS_PER_SECOND_SQUARED = 512;

}
