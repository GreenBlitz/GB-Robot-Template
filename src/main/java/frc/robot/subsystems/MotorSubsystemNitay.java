package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;

public class MotorSubsystemNitay {

	ControllableMotor motor;
	String logPath;
	IRequest<Rotation2d> positionRequest;
	IRequest<Rotation2d> velocityRequest;
	InputSignal<Rotation2d> positionSignal;
	InputSignal<Rotation2d> velocitySignal;
	Rotation2d targetPosition;
	Rotation2d targetVelocity;

	public MotorSubsystemNitay(ControllableMotor motor, String logPath) {
		this.motor = motor;
		this.logPath = logPath;
	}


}
