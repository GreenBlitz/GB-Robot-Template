package frc.robot.subsystems.motorposition;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.Conversions;

public class MotorSubsystemPosition extends GBSubsystem {

	ControllableMotor motor;
	MotorCommandBuilder commandBuilder;
	IRequest<Rotation2d> positionRequest;
	InputSignal<Rotation2d> positionSignal;
	Rotation2d targetPosition;

	public MotorSubsystemPosition(ControllableMotor motor, String logPath) {
		super(logPath);
		this.motor = motor;
		this.commandBuilder = new MotorCommandBuilder();
	}

	public Rotation2d getPosition() {
		if (positionSignal == null) {
			throw new NullPointerException("the position request is null, try using: '.withPositionControl'");
		}
		return positionSignal.getLatestValue();
	}

	public void setTargetPosition(Rotation2d targetPosition) {
		if (positionRequest == null) {
			throw new NullPointerException("the position request is null, try using: '.withPositionControl'");
		}
		this.positionRequest.withSetPoint(targetPosition);
	}

	public boolean isAtPosition(Rotation2d targetPosition, Rotation2d tolerance) {
		return MathUtil.isNear(targetPosition.getRotations(), getPosition().getRotations(), tolerance.getRotations());
	}

	public void resetPosition(Rotation2d resetPosition) {
		motor.resetPosition(resetPosition);
	}

	public double convertToMeters(Rotation2d motorPosition, double wheelDiameter) {
		return Conversions.angleToDistance(motorPosition, wheelDiameter);
	}

	public boolean isPast(Rotation2d expectedPosition) {
		return getPosition().getRadians() > expectedPosition.getRadians();
	}

	protected void stayInPlace() {
		setTargetPosition(getPosition());
	}

	public boolean isBehind(Rotation2d expectedPosition) {
		return getPosition().getRadians() < expectedPosition.getRadians();
	}

	public MotorSubsystemPosition withPositionControl(IRequest<Rotation2d> positionRequest, InputSignal<Rotation2d> positionSignal) {
		this.positionRequest = positionRequest;
		this.positionSignal = positionSignal;
		this.targetPosition = positionSignal.getLatestValue();
		return this;
	}

}
