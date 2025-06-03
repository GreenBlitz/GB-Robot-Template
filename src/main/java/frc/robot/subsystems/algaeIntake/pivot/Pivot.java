package frc.robot.subsystems.algaeIntake.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class Pivot extends GBSubsystem {

	private final ControllableMotor pivot;

	private final IRequest<Rotation2d> positionRequest;

	private final InputSignal<Rotation2d> positionSignal;
	private final InputSignal<Double> voltageSignal;

	private final PivotCommandsBuilder commandsBuilder;

	public Pivot(
		String logPath,
		ControllableMotor pivot,
		IRequest<Rotation2d> positionRequest,
		InputSignal<Rotation2d> positionSignal,
		InputSignal<Double> voltageSignal
	) {
		super(logPath);

		this.pivot = pivot;

		this.positionRequest = positionRequest;

		this.positionSignal = positionSignal;
		this.voltageSignal = voltageSignal;

		this.commandsBuilder = new PivotCommandsBuilder(this);
		pivot.resetPosition(Rotation2d.fromDegrees(110));
		periodic();

		setDefaultCommand(commandsBuilder.stayInPlace());
	}

	public PivotCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public Rotation2d getPosition() {
		return positionSignal.getLatestValue();
	}

	public double getVoltage() {
		return voltageSignal.getLatestValue();
	}

	public boolean isAtPosition(Rotation2d targetPosition, Rotation2d tolerance) {
		return positionSignal.isNear(targetPosition, tolerance);
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
		Logger.recordOutput("aaa", positionRequest.getSetPoint());
	}

	private void updateInputs() {
		pivot.updateSimulation();
		pivot.updateInputs(positionSignal, voltageSignal);
	}

	public void setBrake(boolean brake) {
		pivot.setBrake(brake);
	}

	protected void setTargetPosition(Rotation2d targetPosition) {
		pivot.applyRequest(positionRequest.withSetPoint(targetPosition));
	}

	protected void stayInPlace() {
		pivot.applyRequest(positionRequest.withSetPoint(getPosition()));
	}

}
