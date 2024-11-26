package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.utils.Conversions;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class MotorSubsystemMaya extends GBSubsystem {

	ControllableMotor motor;
	MotorCommandBuilder commandBuilder;
	IRequest<Rotation2d> positionRequest;
	InputSignal<Rotation2d> positionSignal;
	IRequest<Double> voltageRequest;
	SysIdCalibrator sysIdCalibrator;
	Rotation2d targetPosition;

	public MotorSubsystemMaya(ControllableMotor motor, String logPath) {
		super(logPath);
		this.motor = motor;
		this.commandBuilder = new MotorCommandBuilder();
		this.sysIdCalibrator = new SysIdCalibrator(motor.getSysidConfigInfo(), this, this::setVoltage);
	}

	public Rotation2d getPosition() {
		if (positionSignal == null) {
			throw new NullPointerException("the position request is null, try using: '.withPositionControl'");
		}
		return positionSignal.getLatestValue();
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
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

	public MotorCommandBuilder getCommandsBuilder() {
		return commandBuilder;
	}

	public void setPower(double power) {
		motor.setPower(power);
	}

	public void resetPosition(Rotation2d resetPosition) {
		motor.resetPosition(resetPosition);
	}

	public SysIdCalibrator getSysIdCalibrator() {
		return sysIdCalibrator;
	}

	public void setVoltage(double voltage) {
		if (voltageRequest == null) {
			throw new NullPointerException("the voltage request is null, try using: '.withVoltageControl'");
		}
		voltageRequest.withSetPoint(voltage);
	}

	public double convertToMeters(Rotation2d motorPosition, double wheelDiameter) {
		return Conversions.angleToDistance(motorPosition, wheelDiameter);
	}

	public boolean isPast(Rotation2d expectedPosition) {
		return getPosition().getRadians() > expectedPosition.getRadians();
	}

	public boolean isBehind(Rotation2d expectedPosition) {
		return getPosition().getRadians() < expectedPosition.getRadians();
	}

	public MotorSubsystemMaya withPositionControl(IRequest<Rotation2d> positionRequest, InputSignal<Rotation2d> positionSignal) {
		this.positionRequest = positionRequest;
		this.positionSignal = positionSignal;
		this.targetPosition = positionSignal.getLatestValue();
		return this;
	}

	@Override
	protected void subsystemPeriodic() {
		motor.updateInputs();
	}

}
