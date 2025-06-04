package frc.robot.subsystems.algaeIntake.rollers;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;

public class Rollers extends GBSubsystem {

	private final ControllableMotor rollers;

	private final IRequest<Double> voltageRequest;

	private final InputSignal<Rotation2d> velocitySignal;
	private final InputSignal<Double> voltageSignal;

	private final RollersCommandsBuilder commandsBuilder;

	public Rollers(
		String logPath,
		ControllableMotor rollers,
		IRequest<Double> voltageRequest,
		InputSignal<Rotation2d> velocitySignal,
		InputSignal<Double> voltageSignal
	) {
		super(logPath);

		this.rollers = rollers;

		this.voltageRequest = voltageRequest;

		this.velocitySignal = velocitySignal;
		this.voltageSignal = voltageSignal;

		this.commandsBuilder = new RollersCommandsBuilder(this);

		periodic();

		setDefaultCommand(commandsBuilder.stop());
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
		rollers.updateSimulation();
		rollers.updateInputs(velocitySignal, voltageSignal);
	}

	public void setBrake(boolean brake) {
		rollers.setBrake(brake);
	}

	protected void setPower(double power) {
		rollers.setPower(power);
	}

	protected void stop() {
		setPower(0);
	}

	protected void setVoltage(double voltage) {
		rollers.applyRequest(voltageRequest.withSetPoint(voltage));
	}

}
