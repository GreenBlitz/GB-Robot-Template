package frc.robot.subsystems.intake.pivot;

import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.hardware.signal.InputSignal;

public record PivotStuff(
	String logPath,
	ControllableMotor motor,
	InputSignal<Double> voltageSignal,
	InputSignal<Rotation2d> positionSignal,
	IRequest<Double> voltageRequest,
	IRequest<Rotation2d> positionRequest,
	AbsoluteEncoder absoluteEncoder
) {}
