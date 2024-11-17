package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;

public record PivotStuff(
	String logPath,
	ControllableMotor motor,
	IRequest<Rotation2d> positionRequest,
	InputSignal<Rotation2d> positionSignal,
	InputSignal... inputSignals
) {}
