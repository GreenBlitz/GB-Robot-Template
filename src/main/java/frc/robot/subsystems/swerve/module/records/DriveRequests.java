package frc.robot.subsystems.swerve.module.records;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;

public record DriveRequests(
	IRequest<Rotation2d> velocityRequest,
	IRequest<Double> voltageRequest,
	InputSignal<Rotation2d> positionSignal,
	InputSignal<Rotation2d> velocitySignal,
	InputSignal<Double> currentSignal,
	InputSignal<Double> voltageSignal
) {}
