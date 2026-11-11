package frc.robot.subsystems.arm;

import frc.robot.hardware.interfaces.InputSignal;
import org.wpilib.math.geometry.Rotation2d;

public record ArmSignals(
    InputSignal<Double> voltage,
    InputSignal<Double> statorCurrent,
    InputSignal<Double> torqueCurrent,
    InputSignal<Rotation2d> velocity,
    InputSignal<Rotation2d> position) {}
