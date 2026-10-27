package frc.robot.subsystems.roller;

import org.wpilib.math.geometry.Angle2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class Roller extends GBSubsystem {

	protected final ControllableMotor motor;

	private final InputSignal<Double> voltageSignal;
	private final InputSignal<Double> currentSignal;
	private final InputSignal<Angle2d> positionSignal;
	private final InputSignal<Angle2d> velocitySignal;

	private final IRequest<Double> voltageRequest;
	private final RollerCommandsBuilder commandsBuilder;
	private Angle2d targetPosition;

	public Roller(
		String logPath,
		ControllableMotor motor,
		InputSignal<Double> voltageSignal,
		InputSignal<Double> currentSignal,
		InputSignal<Angle2d> positionSignal,
		InputSignal<Angle2d> velocitySignal,
		IRequest<Double> voltageRequest
	) {
		super(logPath);
		this.motor = motor;
		this.voltageSignal = voltageSignal;
		this.currentSignal = currentSignal;
		this.positionSignal = positionSignal;
		this.velocitySignal = velocitySignal;
		this.voltageRequest = voltageRequest;
		this.commandsBuilder = new RollerCommandsBuilder(this);
		this.motor.resetPosition(Angle2d.fromRotations(0));
		this.targetPosition = Angle2d.fromRotations(0);
		setDefaultCommand(commandsBuilder.stop());
	}

	public RollerCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public void updateTargetPosition(Angle2d targetPosition) {
		this.targetPosition = targetPosition;
	}

	public void setVoltage(double voltage) {
		motor.applyRequest(voltageRequest.withSetPoint(voltage));
	}

	public void setPower(double power) {
		motor.setPower(power);
	}

	public void stop() {
		motor.stop();
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	public Double getVoltage() {
		return voltageSignal.getLatestValue();
	}

	public Double getCurrent() {
		return currentSignal.getLatestValue();
	}

	public Angle2d getPosition() {
		return positionSignal.getLatestValue();
	}

	public Angle2d getVelocity() {
		return velocitySignal.getLatestValue();
	}

	public boolean isAtPosition(Angle2d position, Angle2d tolerance) {
		return positionSignal.isNear(position, tolerance);
	}

	public boolean isBehindPosition(Angle2d position) {
		return positionSignal.isLess(position);
	}

	public boolean isPastPosition(Angle2d position) {
		return positionSignal.isGreater(position);
	}

	public boolean isPastTargetPosition() {
		return isPastPosition(targetPosition);
	}

	public boolean isBehindTargetPosition() {
		return isBehindPosition(targetPosition);
	}

	public void update() {
		motor.updateSimulation();
		motor.updateInputs(voltageSignal, currentSignal, positionSignal, velocitySignal);
		Logger.recordOutput(getLogPath() + "/PositionTarget", targetPosition);
	}

}
