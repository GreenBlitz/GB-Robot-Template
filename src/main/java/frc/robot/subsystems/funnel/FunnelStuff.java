package frc.robot.subsystems.funnel;

import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.signal.InputSignal;

public record FunnelStuff(
	String logPath,
	String leftDigitalInputLogPath,
	String rightDigitalInputLogPath,
	IMotor motor,
	InputSignal<Double> inputSignal,
	IDigitalInput leftDigitalInput,
	IDigitalInput rightDigitalInput
) {

	public FunnelStuff(
		String logPath,
		IMotor motor,
		InputSignal<Double> voltageSignal,
		IDigitalInput leftDigitalInput,
		IDigitalInput rightDigitalInput
	) {
		this(logPath, logPath + "leftDigitalInput", logPath + "rightDigitalInput", motor, voltageSignal, leftDigitalInput, rightDigitalInput);
	}

}
