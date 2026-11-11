package frc.robot.subsystems.swerve.module.records;

import frc.robot.hardware.interfaces.InputSignal;
import frc.utils.angle2d.Angle2d;

public record DriveSignals(
    InputSignal<Angle2d> position,
    InputSignal<Angle2d> velocity,
    InputSignal<Double> current,
    InputSignal<Double> voltage) {}
