package frc.robot.subsystems.algaeIntake.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.joysticks.SmartJoystick;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;

public class Pivot extends GBSubsystem {

	private final ControllableMotor pivot;

	private final IRequest<Rotation2d> positionRequest;
	private final InputSignal<Rotation2d> positionSignal;
	private final InputSignal<Double> voltageSignal;
//
//	private final IAngleEncoder encoder;
//	private final InputSignal<Rotation2d> absolutePositionSignal;

	private final PivotCommandsBuilder commandsBuilder;

	public Pivot(
		String logPath,
		ControllableMotor pivot,
		IRequest<Rotation2d> positionRequest,
		InputSignal<Rotation2d> positionSignal,
		InputSignal<Double> voltageSignal
//		IAngleEncoder encoder,
//		InputSignal<Rotation2d> absolutePositionSignal
	) {
		super(logPath);

		this.pivot = pivot;
		this.pivot.resetPosition(PivotConstants.STARTING_POSITION);

		this.positionRequest = positionRequest;

		this.positionSignal = positionSignal;
		this.voltageSignal = voltageSignal;

//		this.encoder = encoder;
//		this.absolutePositionSignal = absolutePositionSignal;

		this.commandsBuilder = new PivotCommandsBuilder(this);

		setDefaultCommand(commandsBuilder.stayInPlace());

		periodic();
	}

	public PivotCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public Rotation2d getPosition() {
		return positionSignal.getLatestValue();
	}

//	public Rotation2d getAbsolutePosition() {
//		return absolutePositionSignal.getLatestValue();
//	}

	public double getVoltage() {
		return voltageSignal.getLatestValue();
	}

	public boolean isAtPosition(Rotation2d targetPosition, Rotation2d tolerance) {
		return positionSignal.isNear(targetPosition, tolerance);
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
		if (getPosition().getDegrees() > PivotConstants.MAX_POSITION.getDegrees()) {
			pivot.resetPosition(PivotConstants.MAX_POSITION);
		}
	}

	private void updateInputs() {
		pivot.updateSimulation();
		pivot.updateInputs(positionSignal, voltageSignal);
//		encoder.updateInputs(absolutePositionSignal);
	}

	public void setBrake(boolean brake) {
		pivot.setBrake(brake);
	}

	public void setTargetPosition(Rotation2d targetPosition) {
		pivot.applyRequest(positionRequest.withSetPoint(targetPosition));
	}

	public void setPower(double power) {
		pivot.setPower(power);
	}

	protected void stayInPlace() {
		pivot.applyRequest(positionRequest.withSetPoint(getPosition()));
	}

	public void applyCalibrationBindings(SmartJoystick joystick) {
		joystick.A.onTrue(commandsBuilder.moveToPosition(PivotState.CLOSED.getPosition()));
		joystick.B.onTrue(commandsBuilder.moveToPosition(PivotState.INTAKE.getPosition()));
		joystick.X.onTrue(commandsBuilder.stayInPlace());
		joystick.Y.onTrue(commandsBuilder.setPower(0.1));
	}

}
