package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.signal.InputSignal;

//@formatter:off
public record ElevatorMotorStuff(
	ControllableMotor motor,
	InputSignal<Double> voltageSignal,
	InputSignal<Rotation2d> positionSignal
) {}
//@formatter:on
