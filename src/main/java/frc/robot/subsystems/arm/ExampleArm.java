package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.ToleranceUtils;

public class ExampleArm extends GBSubsystem {

	public final Rotation2d MAXIMUM_POSITION = Rotation2d.fromDegrees(130);
	public final Rotation2d MINIMUM_POSITION = Rotation2d.fromDegrees(-130);
	public final Rotation2d STARTING_POSITION = Rotation2d.fromDegrees(0);

	private final ControllableMotor motor;
	private final ExampleArmCommandsBuilder commandBuilder;
	private final IRequest<Rotation2d> positionRequest;
	private final IRequest<Double> voltageRequest;
	private final InputSignal<Rotation2d> positionSignal;
	private final InputSignal<Double> voltageSignal;

	public ExampleArm(
		String logPath,
		ControllableMotor motor,
		IRequest<Rotation2d> positionRequest,
		IRequest<Double> voltageRequest,
		InputSignal<Rotation2d> positionSignal,
		InputSignal<Double> voltageSignal
	) {
		super(logPath);
		this.motor = motor;
		this.positionRequest = positionRequest;
		this.voltageRequest = voltageRequest;
		this.positionSignal = positionSignal;
		this.voltageSignal = voltageSignal;
		this.commandBuilder = new ExampleArmCommandsBuilder(this);

		motor.resetPosition(STARTING_POSITION);
		updateInputs();
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
	}

	private void updateInputs() {
		motor.updateInputs(positionSignal, voltageSignal);
	}

	public ExampleArmCommandsBuilder getCommandBuilder() {
		return commandBuilder;
	}

	public Rotation2d getPosition() {
		return positionSignal.getLatestValue();
	}

	public Double getVoltage() {
		return voltageSignal.getLatestValue();
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	protected void setPower(double power) {
		motor.setPower(power);
	}

	protected void setVoltage(double voltage) {
		motor.applyRequest(voltageRequest.withSetPoint(voltage));
	}

	protected void setTargetPosition(Rotation2d angle) {
		motor.applyRequest(positionRequest.withSetPoint(angle));
	}

	protected void stayInPlace() {
		setTargetPosition(positionSignal.getLatestValue());
	}

	public boolean isAtPosition(Rotation2d position, Rotation2d tolerance) {
		return ToleranceUtils.isNearWrapped(position, positionSignal.getLatestValue(), tolerance);
	}

}
