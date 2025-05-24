package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.IRequest;

public record ModuleRequests(
	IRequest<Rotation2d> driveVelocityRequest,
	IRequest<Double> driveVoltageRequest,
	IRequest<Rotation2d> steerAngleRequest
) {}
