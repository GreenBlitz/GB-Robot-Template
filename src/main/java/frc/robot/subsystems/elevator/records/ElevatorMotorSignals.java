package frc.robot.subsystems.elevator.records;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.InputSignal;

public record ElevatorMotorSignals(InputSignal<Rotation2d> positionSignal, InputSignal<Double> voltageSignal, InputSignal<?>... otherSignals) {}
