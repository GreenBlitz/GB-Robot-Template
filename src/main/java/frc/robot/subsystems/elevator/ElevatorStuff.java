package frc.robot.subsystems.elevator;

import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;

public record ElevatorStuff(
	String logPath,
	String digitalInputsLogPath,
	ControllableMotor mainMotor,
	SuppliedDoubleSignal voltageSignal,
	SuppliedAngleSignal mainMotorPositionSignal,
	SuppliedAngleSignal secondaryMotorPositionSignal,
	IDigitalInput digitalInput
) {

	public ElevatorStuff(
		String logPath,
		ControllableMotor mainMotor,
		SuppliedDoubleSignal voltageSignal,
		SuppliedAngleSignal mainMotorPositionSignal,
		SuppliedAngleSignal secondaryMotorPositionSignal,
		IDigitalInput digitalInput
	) {
		this(logPath, logPath + "physicalBreak", mainMotor, voltageSignal, mainMotorPositionSignal, secondaryMotorPositionSignal, digitalInput);
	}

}
