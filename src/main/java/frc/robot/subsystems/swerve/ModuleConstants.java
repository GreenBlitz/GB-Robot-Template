package frc.robot.subsystems.swerve;

import edu.wpi.first.math.util.Units;
import frc.utils.Conversions;

public class ModuleConstants {

    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(1.896924) * 2;
    public static final double MAX_SPEED_REVOLUTIONS_PER_SECOND = Conversions.distanceToRevolutions(SwerveConstants.MAX_SPEED_METERS_PER_SECOND, WHEEL_DIAMETER_METERS);
    public static final double VOLTAGE_COMPENSATION_SATURATION = 12;

    public static final double DRIVE_GEAR_RATIO = 6.12;
    public static final double STEER_GEAR_RATIO = (150.0 / 7.0);

    public static final boolean DEFAULT_IS_DRIVE_MOTOR_CLOSED_LOOP = false;

}
