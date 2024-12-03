package frc.robot.subsystems.motorsubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;

public class MotorSubsystem extends GBSubsystem {

	ControllableMotor motor;
	MotorCommandsBuilder commandBuilder;
	IRequest<Rotation2d> positionRequest;
	InputSignal<Rotation2d> positionSignal;
	MotorSubsystem motorSubsystem;

	public MotorSubsystem(ControllableMotor motor, String logPath) {
		super(logPath);
		this.motor = motor;
		this.commandBuilder = new MotorCommandsBuilder(motorSubsystem);
	}

	public MotorCommandsBuilder getCommandBuilder() {
		return commandBuilder;
	}

	public Rotation2d getPosition() {
		try {
			if (positionSignal == null) {
				throw new NullPointerException("the position request is null, try using: '.withPositionControl'");
			}
			return positionSignal.getLatestValue();
		} catch (NullPointerException e) {
			return null;
		}
	}

	public void setPosition(Rotation2d position) {
		motor.resetPosition(position);
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

	public boolean isPast(Rotation2d expectedPosition) {
		return getPosition().getRadians() > expectedPosition.getRadians();
	}

	public void rollRotations(Rotation2d rotation) {
		setTargetPosition(getPosition().plus(rotation));
	}

	public boolean isBehind(Rotation2d expectedPosition) {
		return getPosition().getRadians() < expectedPosition.getRadians();
	}

	protected void stayInPlace() {
		setTargetPosition(getPosition());
	}

	public MotorSubsystem withPositionControl(IRequest<Rotation2d> positionRequest, InputSignal<Rotation2d> positionSignal) {
		this.positionRequest = positionRequest;
		PositionSignal(positionSignal);
		return this;
	}

	public MotorSubsystem PositionSignal(InputSignal<Rotation2d> positionSignal) {
		this.positionSignal = positionSignal;
		return this;
	}

	public void updateInputs() {
		if (positionSignal != null) {
			motor.updateInputs(positionSignal);
		}
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
	}

}
