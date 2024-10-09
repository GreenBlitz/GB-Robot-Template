package frc.robot.subsystems.elevatorRoller;

import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.signal.InputSignal;

public record ElevatorRollerStuff(
	String logPath,
	String digitalInputLogPath,
	IMotor motor,
	IDigitalInput digitalInput,
	InputSignal<Double> motorVoltage
) {

	public ElevatorRollerStuff(String logPath, IMotor motor, IDigitalInput digitalInput, InputSignal<Double> motorVoltage) {
		this(logPath, logPath + "digitalInput", motor, digitalInput, motorVoltage);
	}

}
