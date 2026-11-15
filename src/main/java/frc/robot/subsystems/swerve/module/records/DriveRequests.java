package frc.robot.subsystems.swerve.module.records;

import frc.robot.hardware.interfaces.IRequest;
import org.wpilib.math.geometry.Rotation2d;

public record DriveRequests(
    IRequest<Rotation2d> velocity, IRequest<Double> voltage, IRequest<Double> torqueCurrent) {}
