package frc.robot.subsystems.elevator.records;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;

public record ElevatorMotor(
	ControllableMotor motor,
	IRequest<Rotation2d> positionRequest,
	IRequest<Double> voltageRequest,
	InputSignal<Rotation2d> positionSignal,
	InputSignal<Double> voltageSignal,
	InputSignal<?>... otherSignals
) {}
