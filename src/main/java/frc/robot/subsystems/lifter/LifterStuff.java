package frc.robot.subsystems.lifter;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.subsystems.swerve.swervestatehelpers.RotateAxis;

public record LifterStuff (
        String logPath,
        ControllableMotor motor,
        InputSignal<Rotation2d> positionSignal,
        IRequest<Double> positionRequest,
        InputSignal... otherSignals
){}