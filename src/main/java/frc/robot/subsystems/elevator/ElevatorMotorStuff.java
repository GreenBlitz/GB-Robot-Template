package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.signal.InputSignal;

public record ElevatorMotorStuff(
        ControllableMotor motor,
        InputSignal<Double> voltageSignal,
        InputSignal<Rotation2d> motorPositionSignal,
) { }
