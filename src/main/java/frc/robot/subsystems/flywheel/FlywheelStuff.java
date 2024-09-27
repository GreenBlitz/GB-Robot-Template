package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.hardware.signal.InputSignal;

public record FlywheelStuff(
	ControllableMotor rightFlywheel,
	ControllableMotor leftFlywheel,
	IRequest<Rotation2d> rightFlywheelVelocityRequest,
	IRequest<Rotation2d> leftFlywheelVelocityRequest,
	InputSignal<Rotation2d> rightVelocitySignal,
	InputSignal<Rotation2d> leftVelocitySignal,
	InputSignal[] rightSignals,
	InputSignal[] leftSignals
) {}
