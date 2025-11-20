package frc.robot.subsystems.roller;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;

public class Roller extends GBSubsystem {

	protected final ControllableMotor roller;

	private final InputSignal<Double> voltageSignal;
	private final InputSignal<Double> currentSignal;
	private final InputSignal<Rotation2d> positionSignal;
	private final IRequest<Double> voltageRequest;
	private final RollerCommandsBuilder commandsBuilder;
	private Rotation2d targetPosition;

	public Roller(
		String logPath,
		ControllableMotor roller,
		InputSignal<Double> voltageSignal,
		InputSignal<Rotation2d> positionSignal,
		InputSignal<Double> currentSignal,
		IRequest<Double> voltageRequest
	) {
		super(logPath);
		this.roller = roller;
		this.voltageSignal = voltageSignal;
		this.currentSignal = currentSignal;
		this.positionSignal = positionSignal;
		this.voltageRequest = voltageRequest;
		this.commandsBuilder = new RollerCommandsBuilder(this);
		this.roller.resetPosition(Rotation2d.fromRotations(0));
		this.targetPosition = Rotation2d.fromRotations(0);
	}

	public RollerCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public void updateTargetPosition(Rotation2d targetPosition) {
		this.targetPosition = targetPosition;
	}

	public void setVoltage(double voltage) {
		roller.applyRequest(voltageRequest.withSetPoint(voltage));
	}

	public void setPower(double power) {
		roller.setPower(power);
	}

	public void stop() {
		roller.stop();
	}

	public void setBrake(boolean brake) {
		roller.setBrake(brake);
	}

	public Double getVoltage() {
		return voltageSignal.getLatestValue();
	}

	public Double getCurrent() {
		return currentSignal.getLatestValue();
	}

	public Rotation2d getPosition() {
		return positionSignal.getLatestValue();
	}

	public boolean isAtPosition(Rotation2d position, Rotation2d tolerance) {
		return positionSignal.isNear(position, tolerance);
	}

	public boolean isBehindPosition(Rotation2d position) {
		return positionSignal.isLess(position);
	}

	public boolean isPastPosition(Rotation2d position) {
		return positionSignal.isGreater(position);
	}

	public boolean isPastTargetPosition() {
		return isPastPosition(targetPosition);
	}

	public boolean isBehindTargetPosition() {
		return isBehindPosition(targetPosition);
	}

	@Override
	public void subsystemPeriodic() {
		roller.updateSimulation();
		roller.updateInputs(voltageSignal, currentSignal, positionSignal);
	}

}
