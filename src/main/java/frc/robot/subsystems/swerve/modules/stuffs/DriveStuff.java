package frc.robot.subsystems.swerve.modules.stuffs;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.hardware.signal.InputSignal;

public record DriveStuff(
	ControllableMotor drive,
	IRequest<Rotation2d> velocityRequest,
	IRequest<Double> voltageRequest,
	InputSignal<Rotation2d> positionSignal,
	InputSignal<Rotation2d> velocitySignal,
	InputSignal<Double> currentSignal,
	InputSignal<Double> voltageSignal
) {}