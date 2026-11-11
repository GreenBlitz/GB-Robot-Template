package frc.robot.subsystems.swerve.module.records;

import frc.robot.hardware.interfaces.InputSignal;
import org.wpilib.math.geometry.Rotation2d;

public record SteerSignals(
    InputSignal<Rotation2d> position,
    InputSignal<Rotation2d> velocity,
    InputSignal<Double> current,
    InputSignal<Double> voltage) {}
