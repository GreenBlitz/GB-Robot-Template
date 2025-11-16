package frc.robot.subsystems.roller;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;


public class Roller extends GBSubsystem {

	private final ControllableMotor roller;

	private final InputSignal<Double> voltageSignal;
	private final InputSignal<Rotation2d> positionSignal;
	private final InputSignal<Double> currentSignal;

	private final IRequest<Double> VoltageRequest;
	private final IRequest<Rotation2d> PositionRequest;

	private final Rotation2d tolerance;

	private final RollerCommandsBuilder commandsBuilder;

	private Rotation2d targetPosition;

	public Roller(
		String logPath,
		ControllableMotor roller,
		InputSignal<Double> voltageSignal,
		InputSignal<Rotation2d> positionSignal,
		InputSignal<Double> currentSignal,
		IRequest<Double> VoltageRequest,
		IRequest<Rotation2d> PositionRequest,
		Rotation2d tolerance
	) {
		super(logPath);
		this.roller = roller;
		this.voltageSignal = voltageSignal;
		this.currentSignal = currentSignal;
		this.positionSignal = positionSignal;
		this.VoltageRequest = VoltageRequest;
		this.PositionRequest = PositionRequest;
		this.tolerance = tolerance;
		this.commandsBuilder = new RollerCommandsBuilder(this);
		this.targetPosition = Rotation2d.fromRotations(0);
	}

	public RollerCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public void updateTargetPosition(Rotation2d targetPosition) {
		this.targetPosition = targetPosition;
	}

	public void setVoltage(double voltage) {
		roller.applyRequest(VoltageRequest.withSetPoint(voltage));
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

	public Rotation2d getTargetPosition() {
		return targetPosition;
	}

	public boolean isAtPosition(Rotation2d position) {
		return positionSignal.isNear(position, tolerance);
	}


	public boolean isBeforePosition(Rotation2d position) {
		return positionSignal.isLess(position);
	}

	public BooleanSupplier isPastPosition(Rotation2d position) {
		return () -> positionSignal.isGreater(position);
	}

	private void log() {
		Logger.recordOutput(getLogPath() + "targetPosition", targetPosition);
	}


	@Override
	public void subsystemPeriodic() {
		roller.updateSimulation();
		log();
		roller.updateInputs(voltageSignal, currentSignal, positionSignal);
	}

}
