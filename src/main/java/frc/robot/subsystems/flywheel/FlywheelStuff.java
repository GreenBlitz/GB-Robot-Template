package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.hardware.signal.InputSignal;

public record FlywheelStuff(
	String logPath,
	ControllableMotor motor,
	boolean isInverted,
	InputSignal<Double> voltageSignal,
	InputSignal<Rotation2d> velocitySignal,
	IRequest<Double> voltageRequest,
	IRequest<Rotation2d> velocityRequest
) {}
