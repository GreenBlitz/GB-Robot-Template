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

	private final RollersCommandsBuilder commandsBuilder;

	public Rollers(
		String logPath,
		ControllableMotor rollers,
		IRequest<Rotation2d> rollersVelocityRequest,
		IRequest<Double> rollersVoltageRequest,
		InputSignal<Rotation2d> rollersVelocitySignal,
		InputSignal<Double> rollersVoltageSignal
	) {
		super(logPath);

		this.rollers = rollers;

		this.velocityRequest = rollersVelocityRequest;
		this.voltageRequest = rollersVoltageRequest;

		this.velocitySignal = rollersVelocitySignal;
		this.voltageSignal = rollersVoltageSignal;

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

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
	}

	private void updateInputs() {
		rollers.updateInputs(velocitySignal, voltageSignal);
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
