package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.request.IRequest;

public record ElevatorStuff(
	String logPath,
	String digitalInputsLogPath,
	IRequest<Rotation2d> angleRequest,
	IDigitalInput digitalInput,
	ElevatorMotorStuff frontMotorStuff,
	ElevatorMotorStuff backMotorStuff,
	double motorRotationsToMetersConversionRatio
) {

	public ElevatorStuff(
		String logPath,
		IRequest<Rotation2d> angleRequest,
		IDigitalInput digitalInput,
		ElevatorMotorStuff frontMotorStuff,
		ElevatorMotorStuff backMotorStuff,
		double rotationsToMetersConversionRatio
	) {
		this(logPath, logPath + "limitSwitch", angleRequest, digitalInput, frontMotorStuff, backMotorStuff, rotationsToMetersConversionRatio);
	}

}
