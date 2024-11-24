package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.utils.Conversions;

public class MotorSubsystemNitay extends GBSubsystem {

	ControllableMotor motor;
	MotorCommandBuilder commandBuilder;
	IRequest<Rotation2d> positionRequest;
	IRequest<Rotation2d> velocityRequest;
	IRequest<Double> volatgeRequest;
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

	public MotorSubsystemNitay withVoltageControl(IRequest<Double> voltageRequest) {
		this.volatgeRequest = voltageRequest;
		return this;
	}

	public Rotation2d getVelocityRotation2dPerSecond(){
		if (velocitySignal==null){
			throw  new NullPointerException("the velocity request is null, try using: '.withVelocityControl'");
		}
		return velocitySignal.getLatestValue();
	}

	public void setTargetVelocityRotation2dPerSecond(Rotation2d targetVelocity){
		if (velocityRequest==null){
			throw  new NullPointerException("the velocity request is null, try using: '.withVelocityControl'");
		}
		this.velocityRequest.withSetPoint(targetVelocity);
	}

	public void updateInputs(){
		motor.updateInputs(positionSignal, velocitySignal);
	}

	public void stop(){
		motor.stop();
	}

	public void setVoltage(double voltage){
		if (volatgeRequest==null){
			throw  new NullPointerException("the voltage request is null, try using: '.withVoltageControl'");
		}
		this.volatgeRequest.withSetPoint(voltage);
	}

	public void setPosition(Rotation2d position){
		motor.resetPosition(position);
	}

	private Rotation2d convertFromMeters(double positionInMeters, double wheelDiameter){
		return Conversions.distanceToAngle(positionInMeters, wheelDiameter);
	}

	public String getLogPath(){
		return super.getLogPath();
	}

	public void applyRequests(){
		motor.applyRequest(positionRequest);
		motor.applyRequest(velocityRequest);
		motor.applyRequest(volatgeRequest);
	}


}
