package frc.robot.subsystems.algaeIntake.rollers;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;

public class Rollers extends GBSubsystem {

	private final ControllableMotor rollers;

	private final IRequest<Rotation2d> velocityRequest;
	private final IRequest<Double> voltageRequest;

	private final InputSignal<Rotation2d> velocitySignal;
	private final InputSignal<Double> voltageSignal;
	private final InputSignal<Double> currentSignal;

	private final RollersCommandsBuilder commandsBuilder;

	public Rollers(
		String logPath,
		ControllableMotor rollers,
		IRequest<Rotation2d> velocityRequest,
		IRequest<Double> voltageRequest,
		InputSignal<Rotation2d> velocitySignal,
		InputSignal<Double> voltageSignal,
		InputSignal<Double> currentSignal
	) {
		super(logPath);

		this.rollers = rollers;

		this.velocityRequest = velocityRequest;
		this.voltageRequest = voltageRequest;

		this.velocitySignal = velocitySignal;
		this.voltageSignal = voltageSignal;
		this.currentSignal = currentSignal;

		periodic();

		this.commandsBuilder = new RollersCommandsBuilder(this);
	}

	public RollersCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public Rotation2d getVelocityRotation2dPerSecond() {
		return velocitySignal.getLatestValue();
	}

	public double getVoltage() {
		return voltageSignal.getLatestValue();
	}

	public double getCurrent() {
		return currentSignal.getLatestValue();
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
	}

	private void updateInputs() {
		rollers.updateInputs(velocitySignal, voltageSignal, currentSignal);
		rollers.updateSimulation();
	}

	public void setBrake(boolean brake) {
		rollers.setBrake(brake);
	}

	protected void setTargetVelocity(Rotation2d targetVelocityMPS) {
		rollers.applyRequest(velocityRequest.withSetPoint(targetVelocityMPS));
	}

	protected void setVoltage(double voltage) {
		rollers.applyRequest(voltageRequest.withSetPoint(voltage));
	}

}
