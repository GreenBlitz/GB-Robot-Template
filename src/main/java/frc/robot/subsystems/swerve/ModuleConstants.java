package frc.robot.subsystems.swerve;

import frc.utils.Conversions;

public class ModuleConstants {

    public static final double WHEEL_DIAMETER_METERS = 0.1016;
    public static final double MAX_SPEED_REVOLUTIONS_PER_SECOND = Conversions.distanceToRevolutions(SwerveConstants.MAX_SPEED_METERS_PER_SECOND, WHEEL_DIAMETER_METERS);
    public static final double VOLTAGE_COMPENSATION_SATURATION = 12;

    public static final double DRIVE_GEAR_RATIO = 6.75;
    public static final double STEER_GEAR_RATIO = 12.8;

}
