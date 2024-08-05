package frc.robot.subsystems.swerve.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.Conversions;
import frc.utils.battery.BatteryUtils;

public record ModuleConstants(
        String modulesLogPath,
        String alertLogPath,
        double wheelDiameterMeters,
        double couplingRatio,
        Rotation2d velocityAt12VoltsPerSecond
) {

    public ModuleConstants(
            String swerveLogPath,
            String moduleType,
            double wheelDiameterMeters,
            double couplingRatio,
            double velocityAt12VoltsMetersPerSecond) {
        this(
                swerveLogPath + moduleType + "Modules/",
                swerveLogPath + moduleType + "Modules/Alerts/",
                wheelDiameterMeters,
                couplingRatio,
                Conversions.distanceToAngle(
                        velocityAt12VoltsMetersPerSecond,
                        wheelDiameterMeters
                )
        );
    }

    static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(3);
    static final Rotation2d ANGLE_VELOCITY_DEADBAND = Rotation2d.fromDegrees(3);
    static final double SPEED_TOLERANCE_METERS_PER_SECOND = 0.1;

    public static final double VOLTAGE_COMPENSATION_SATURATION = BatteryUtils.DEFAULT_VOLTAGE;

}
