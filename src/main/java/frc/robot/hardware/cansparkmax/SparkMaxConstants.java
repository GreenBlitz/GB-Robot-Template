package frc.robot.hardware.cansparkmax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utils.calibration.sysid.SysIdCalibrator;

import java.util.function.BiFunction;

public record SparkMaxConstants(BiFunction<Rotation2d, Rotation2d, Rotation2d> feedforward, SysIdCalibrator.SysIdConfigInfo sysIdConfigInfo) {

    public SparkMaxConstants(BiFunction<Rotation2d, Rotation2d, Rotation2d> feedforward, SysIdRoutine.Config sysidConfig) {
        this(feedforward, new SysIdCalibrator.SysIdConfigInfo(sysidConfig, false));
    }

}
