package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.hardware.signal.InputSignal;

public record BottomFlywheelComponents(
        String logPath,
        ControllableMotor motor,
        boolean isMotorInverted,
        InputSignal<Double> motorVoltageSignal,
        InputSignal<Rotation2d> motorVelocitySignal,
        IRequest<Rotation2d> motorVelocityRequest

) {}
