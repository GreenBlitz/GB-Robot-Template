package frc.robot.subsystems.intake;

import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.signal.InputSignal;

public record IntakeStuff(String logPath, String digitalInputLogPath, IMotor motor, InputSignal<Double> voltageSignal, IDigitalInput digitalInput) {

    public IntakeStuff(String logPath, IMotor motor, InputSignal<Double> voltageSignal, IDigitalInput digitalInput) {
        this(logPath, logPath + "digitalInput", motor, voltageSignal, digitalInput);
    }

}
