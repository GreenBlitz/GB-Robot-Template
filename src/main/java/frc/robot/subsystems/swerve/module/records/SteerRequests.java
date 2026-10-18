package frc.robot.subsystems.swerve.module.records;

import org.wpilib.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.IRequest;

public record SteerRequests(IRequest<Rotation2d> position, IRequest<Double> voltage) {}
