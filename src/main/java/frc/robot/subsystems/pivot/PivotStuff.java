package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.hardware.signal.InputSignal;

public record PivotStuff(
	ControllableMotor motor,
	IRequest<Rotation2d> positionRequest,
	InputSignal<Rotation2d> positionSignal,
	InputSignal... inputSignals
) {}
