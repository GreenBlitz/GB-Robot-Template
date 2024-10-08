package frc.robot.subsystems.intake.roller;

import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.signal.InputSignal;

public record IntakeRollerStuff(
	String logPath,
	String digitalInputLogPath,
	IMotor motor,
	InputSignal<Double> voltageSignal,
	IDigitalInput digitalInput
) {

	//@formatter:off
	public IntakeRollerStuff(
		String logPath,
		IMotor motor,
		InputSignal<Double> voltageSignal,
		IDigitalInput digitalInput
	) {
		this(logPath, logPath + "digitalInput/", motor, voltageSignal, digitalInput);
	}
	//@formatter:on

}
