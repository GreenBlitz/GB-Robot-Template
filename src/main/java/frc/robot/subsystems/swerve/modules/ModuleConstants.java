package frc.robot.subsystems.swerve.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.utils.Conversions;

public class ModuleConstants {

    public static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(3);
    public static final double SPEED_TOLERANCE_METERS_PER_SECOND = 0.1;

    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(1.896924) * 2;

    public static final Rotation2d MAX_SPEED_PER_SECOND = Rotation2d.fromRotations(Conversions.distanceToRevolutions(
            SwerveConstants.MAX_SPEED_METERS_PER_SECOND,
            WHEEL_DIAMETER_METERS
    ));

    public static final double VOLTAGE_COMPENSATION_SATURATION = 12; // todo add battery class

    public static final double DRIVE_GEAR_RATIO = 6.12;

    public static final double STEER_GEAR_RATIO = (150.0 / 7.0);

    public static final boolean DEFAULT_IS_DRIVE_MOTOR_CLOSED_LOOP = false;

}
