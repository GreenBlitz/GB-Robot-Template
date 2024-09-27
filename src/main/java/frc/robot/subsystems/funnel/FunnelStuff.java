package frc.robot.subsystems.funnel;

import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.signal.InputSignal;

public record FunnelStuff(
	String logPath,
	String digitalInputLogPath,
	IMotor motor,
	InputSignal<Double> inputSignal,
	IDigitalInput digitalInput
) {

	public FunnelStuff(String logPath, IMotor motor, InputSignal<Double> voltageSignal, IDigitalInput digitalInput) {
		this(logPath, logPath + "digitalInput", motor, voltageSignal, digitalInput);
	}

}
