package frc.robot.subsystems.swerve.module.records;

import frc.utils.angle2d.Angle2d;
import frc.robot.hardware.interfaces.IRequest;

public record SteerRequests(IRequest<Angle2d> position, IRequest<Double> voltage) {}
