package frc.robot.subsystems.swerve.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.LogPathsConstants;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.utils.battery.BatteryUtils;

public class ModuleConstants {

    protected static String LOG_PATH = SwerveConstants.SWERVE_LOG_PATH + "Modules/";
    protected static String ALERT_LOG_PATH = LogPathsConstants.ALERT_LOG_PATH + LOG_PATH;

    public static final double VOLTAGE_COMPENSATION_SATURATION = BatteryUtils.DEFAULT_VOLTAGE;

    public static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(3);
    public static final Rotation2d ANGLE_VELOCITY_DEADBAND = Rotation2d.fromDegrees(3);
    public static final double SPEED_TOLERANCE_METERS_PER_SECOND = 0.1;



}
