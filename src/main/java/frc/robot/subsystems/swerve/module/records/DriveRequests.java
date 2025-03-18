package frc.robot.subsystems.swerve.module.records;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.IRequest;

public record DriveRequests(IRequest<Rotation2d> velocity, IRequest<Double> voltage, IRequest<Double> torqueCurrent) {}
