package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;

public class MotorSubsystemNitay extends GBSubsystem {

	ControllableMotor motor;
	String logPath;
	MotorCommandBuilder commandBuilder;
	IRequest<Rotation2d> positionRequest;
	IRequest<Rotation2d> velocityRequest;
	InputSignal<Rotation2d> positionSignal;
	InputSignal<Rotation2d> velocitySignal;
	Rotation2d targetPosition;
	Rotation2d targetVelocity;

	public MotorSubsystemNitay(ControllableMotor motor, String logPath) {
		super(logPath);
		this.motor = motor;
		this.commandBuilder = new MotorCommandBuilder();
	}

	public MotorSubsystemNitay withVelocityControl(IRequest<Rotation2d> velocityRequest, InputSignal<Rotation2d> velocitySignal) {
		this.velocityRequest = velocityRequest;
		this.velocitySignal = velocitySignal;
		this.targetVelocity = velocitySignal.getLatestValue();
		return this;
	}

	public Rotation2d getVelocityRotation2dPerSecond(){
		return velocitySignal.getLatestValue();
	}

	public void setTargetVelocityRotation2dPerSecond(Rotation2d targetVelocity){
		this.targetVelocity = targetVelocity;
	}

	public void updateInputs(){
		motor.updateInputs(positionSignal, velocitySignal);
	}

	public void stop(){
		motor.stop();
	}

}
