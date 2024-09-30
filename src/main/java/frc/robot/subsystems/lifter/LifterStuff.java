package frc.robot.subsystems.lifter;

import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.hardware.signal.InputSignal;

public record LifterStuff (
        String logPath,
        ControllableMotor motor,
        InputSignal<Double> positionSignal,
        IRequest<Double> positionRequest,
        InputSignal... otherSignals
){}