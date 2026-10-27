package frc.robot.subsystems.swerve.module.records;

import org.wpilib.math.geometry.Angle2d;
import frc.robot.hardware.interfaces.IRequest;

public record SteerRequests(IRequest<Angle2d> position, IRequest<Double> voltage) {}
