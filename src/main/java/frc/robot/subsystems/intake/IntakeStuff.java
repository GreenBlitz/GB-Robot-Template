package frc.robot.subsystems.intake;

import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.signal.InputSignal;

public record IntakeStuff(IMotor motor, InputSignal<Double> inputSignal, IDigitalInput digitalInput) {}
