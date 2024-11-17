package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;

public record FlywheelStuff(
	String logPath,
	ControllableMotor rightFlywheel,
	ControllableMotor leftFlywheel,
	IRequest<Rotation2d> rightFlywheelVelocityRequest,
	IRequest<Rotation2d> leftFlywheelVelocityRequest,
	IRequest<Double> rightVoltageRequest,
	IRequest<Double> leftVoltageRequest,
	InputSignal<Rotation2d> rightVelocitySignal,
	InputSignal<Rotation2d> leftVelocitySignal,
	InputSignal[] rightSignals,
	InputSignal[] leftSignals
) {}
