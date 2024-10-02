package frc.robot.subsystems.elevator;

import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.cansparkmax.SparkMaxAngleRequest;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;

import java.util.function.Function;

public record ElevatorStuff(
	String logPath,
	String digitalInputsLogPath,
	ControllableMotor mainMotor,
	SuppliedDoubleSignal voltageSignal,
	SuppliedAngleSignal mainMotorPositionSignal,
	SparkMaxAngleRequest angleRequest,
	IDigitalInput digitalInput
) {

	public ElevatorStuff(
		String logPath,
		ControllableMotor mainMotor,
		SuppliedDoubleSignal voltageSignal,
		SuppliedAngleSignal mainMotorPositionSignal,
		SparkMaxAngleRequest angleRequest,
		IDigitalInput digitalInput
	) {
		this(logPath, logPath + "limitSwitch", mainMotor, voltageSignal, mainMotorPositionSignal, angleRequest, digitalInput);
	}

}
