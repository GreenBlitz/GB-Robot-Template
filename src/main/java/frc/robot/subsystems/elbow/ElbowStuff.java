package frc.robot.subsystems.elbow;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.hardware.signal.InputSignal;

public record ElbowStuff(
	ControllableMotor elbow,
	IRequest<Rotation2d> positionRequest,
	IRequest<Double> voltageRequest,
	InputSignal<Rotation2d> positionSignal,
	InputSignal<Rotation2d> velocitySignal,
	InputSignal<Rotation2d> currentSignal,
	InputSignal<Rotation2d> voltageSignal
) {}
