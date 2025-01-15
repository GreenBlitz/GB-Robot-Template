package frc.robot.subsystems.elevator.records;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.IRequest;

public record ElevatorRequests(IRequest<Rotation2d> positionRequest, IRequest<Double> voltageRequest) {}
