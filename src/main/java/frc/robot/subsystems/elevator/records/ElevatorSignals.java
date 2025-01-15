package frc.robot.subsystems.elevator.records;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.InputSignal;

public record ElevatorSignals(InputSignal<Rotation2d> positionSignal, InputSignal<Double> voltageSignal, InputSignal<?>... otherSignals) {}
