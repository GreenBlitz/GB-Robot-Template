package frc.robot.subsystems.funnel;

import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.signal.InputSignal;

public record FunnelStuff(
	String logPath,
	String shooterDigitalInputLogPath,
	IMotor motor,
	InputSignal<Double> voltageSignal,
	IDigitalInput shooterDigitalInput

) {

	public FunnelStuff(String logPath, IMotor motor, InputSignal<Double> voltageSignal, IDigitalInput shooterDigitalInput) {
		this(logPath, logPath + "shooterDigitalInput/", motor, voltageSignal, shooterDigitalInput);
	}

}
