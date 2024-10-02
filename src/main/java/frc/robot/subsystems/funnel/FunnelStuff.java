package frc.robot.subsystems.funnel;

import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.signal.InputSignal;

public record FunnelStuff(
	String logPath,
	String shooterDigitalInputLogPath,
	String elevatorDigitalInputLogPath,
	IMotor motor,
	InputSignal<Double> voltageSignal,
	IDigitalInput shooterDigitalInput,
	IDigitalInput elevatorDigitalInput

) {

	public FunnelStuff(
		String logPath,
		IMotor motor,
		InputSignal<Double> voltageSignal,
		IDigitalInput shooterDigitalInput,
		IDigitalInput elevatorDigitalInput
	) {
		this(
			logPath,
			logPath + "shooterDigitalInput",
			logPath + "ampDigitalInput",
			motor,
			voltageSignal,
			shooterDigitalInput,
			elevatorDigitalInput
		);
	}

}
