package frc.robot.subsystems.swerve.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.LogPaths;
import frc.utils.Conversions;
import frc.utils.battery.BatteryUtils;

public record ModuleConstants(String modulesLogPath, double wheelDiameterMeters, double couplingRatio,
        Rotation2d velocityAt12VoltsPerSecond) {

    public ModuleConstants(String swerveLogPath, String modulesName, double wheelDiameterMeters, double couplingRatio,
            double velocityAt12VoltsMetersPerSecond){
        this(swerveLogPath + modulesName + "Modules/", wheelDiameterMeters, couplingRatio,
                Conversions.distanceToAngle(velocityAt12VoltsMetersPerSecond,
                wheelDiameterMeters));
    }

    String getAlertLogPath() {
        return LogPaths.ALERT_LOG_PATH + modulesLogPath();
    }

    static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(3);
    static final Rotation2d ANGLE_VELOCITY_DEADBAND = Rotation2d.fromDegrees(3);
    static final double SPEED_TOLERANCE_METERS_PER_SECOND = 0.1;

    public static final double VOLTAGE_COMPENSATION_SATURATION = BatteryUtils.DEFAULT_VOLTAGE;

}
