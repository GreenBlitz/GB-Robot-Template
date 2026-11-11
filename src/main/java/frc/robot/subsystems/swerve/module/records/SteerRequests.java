package frc.robot.subsystems.swerve.module.records;

import frc.robot.hardware.interfaces.IRequest;
import org.wpilib.math.geometry.Rotation2d;

public record SteerRequests(IRequest<Rotation2d> position, IRequest<Double> voltage) {}
