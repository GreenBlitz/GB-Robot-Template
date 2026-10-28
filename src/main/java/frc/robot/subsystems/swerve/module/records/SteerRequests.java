package frc.robot.subsystems.swerve.module.records;


import frc.robot.hardware.interfaces.IRequest;
import frc.utils.costomUnits.Angle2d;

public record SteerRequests(IRequest<Angle2d> position, IRequest<Double> voltage) {}
