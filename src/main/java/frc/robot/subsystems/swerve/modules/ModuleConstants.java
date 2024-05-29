package frc.robot.subsystems.swerve.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.LogPathsConstants;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.utils.Conversions;
import frc.utils.batteryutils.Battery;

public class ModuleConstants {

    protected static String LOG_PATH = SwerveConstants.SWERVE_LOG_PATH + "Modules/";
    protected static String ALERT_LOG_PATH = LogPathsConstants.ALERT_LOG_PATH + LOG_PATH;

    public static final boolean ENABLE_FOC_DRIVE = true;
    public static final boolean ENABLE_FOC_STEER = true;

    public static final boolean DEFAULT_IS_DRIVE_MOTOR_CLOSED_LOOP = false;
    public static final double VOLTAGE_COMPENSATION_SATURATION = Battery.getDefaultBatteryVoltage();

    public static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(3);
    public static final Rotation2d ANGLE_VELOCITY_TOLERANCE = Rotation2d.fromDegrees(3);
    public static final double SPEED_TOLERANCE_METERS_PER_SECOND = 0.1;

    public static final double DRIVE_GEAR_RATIO = 6.12;
    public static final double STEER_GEAR_RATIO = (150.0 / 7.0);

    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(1.896924) * 2;//todo - calibrate

    public static final Rotation2d MAX_SPEED_PER_SECOND = Rotation2d.fromRotations(Conversions.distanceToRevolutions(
            SwerveConstants.MAX_SPEED_METERS_PER_SECOND,
            WHEEL_DIAMETER_METERS
    ));

}
