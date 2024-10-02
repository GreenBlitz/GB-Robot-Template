package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.hardware.request.cansparkmax.SparkMaxAngleRequest;
import frc.robot.hardware.signal.InputSignal;


public record ElevatorStuff(
	String logPath,
	String digitalInputsLogPath,
	ControllableMotor mainMotor,
	InputSignal<Double> voltageSignal,
	InputSignal<Rotation2d> mainMotorPositionSignal,
	IRequest<Rotation2d> angleRequest,
	IDigitalInput digitalInput
) {

	public ElevatorStuff(
		String logPath,
		ControllableMotor mainMotor,
		InputSignal<Double> voltageSignal,
		InputSignal<Rotation2d> mainMotorPositionSignal,
		SparkMaxAngleRequest angleRequest,
		IDigitalInput digitalInput
	) {
		this(logPath, logPath + "limitSwitch", mainMotor, voltageSignal, mainMotorPositionSignal, angleRequest, digitalInput);
	}

}
