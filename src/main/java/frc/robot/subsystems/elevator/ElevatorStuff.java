package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.request.IRequest;

public record ElevatorStuff(
	String logPath,
	String digitalInputsLogPath,
	IRequest<Rotation2d> positionRequest,
	IRequest<Double> voltageRequest,
	IDigitalInput digitalInput,
	ElevatorMotorStuff frontMotorStuff,
	ElevatorMotorStuff backMotorStuff
) {

	public ElevatorStuff(
		String logPath,
		IRequest<Rotation2d> positionRequest,
		IRequest<Double> voltageRequest,
		IDigitalInput digitalInput,
		ElevatorMotorStuff frontMotorStuff,
		ElevatorMotorStuff backMotorStuff
	) {
		this(logPath, logPath + "limitSwitch/", positionRequest, voltageRequest, digitalInput, frontMotorStuff, backMotorStuff);
	}

}
