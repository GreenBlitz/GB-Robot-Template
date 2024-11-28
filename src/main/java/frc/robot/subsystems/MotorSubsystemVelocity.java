package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;

public class MotorSubsystemVelocity {

    ControllableMotor motor;
    MotorCommandBuilder commandBuilder;
    IRequest<Rotation2d> positionRequest;
    IRequest<Rotation2d> velocityRequest;
    IRequest<Double> volatgeRequest;
    InputSignal<Rotation2d> positionSignal;
    InputSignal<Rotation2d> velocitySignal;
    Rotation2d targetPosition;
    Rotation2d targetVelocity;

    public MotorSubsystemVelocity(ControllableMotor motor, String logPath) {}

}
