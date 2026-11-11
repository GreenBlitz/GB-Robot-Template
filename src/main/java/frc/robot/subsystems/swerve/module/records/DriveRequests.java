package frc.robot.subsystems.swerve.module.records;

import frc.robot.hardware.interfaces.IRequest;
import frc.utils.angle2d.Angle2d;

public record DriveRequests(
    IRequest<Angle2d> velocity, IRequest<Double> voltage, IRequest<Double> torqueCurrent) {}
