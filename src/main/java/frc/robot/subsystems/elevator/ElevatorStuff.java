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
	ElevatorMotorStuff motorStuff
) {

	public ElevatorStuff(
		String logPath,
		IRequest<Rotation2d> positionRequest,
		IRequest<Double> voltageRequest,
		IDigitalInput digitalInput,
		ElevatorMotorStuff motorStuff
	) {
		this(logPath, logPath + "limitSwitch/", positionRequest, voltageRequest, digitalInput, motorStuff);
	}

}
