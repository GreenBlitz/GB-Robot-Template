package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.math.ToleranceMath;

public class Arm extends GBSubsystem {

	private final ControllableMotor motor;
	private final IRequest<Rotation2d> positionRequest;
	private final IRequest<Double> voltageRequest;
	private final InputSignal<Rotation2d> positionSignal;
	private final InputSignal<Double> voltageSignal;
	private final ArmCommandsBuilder commandsBuilder;

	public Arm(
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
		this.commandsBuilder = new ArmCommandsBuilder(this);

		updateInputs();
	}

	public ArmCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public Rotation2d getPosition() {
		return positionSignal.getLatestValue();
	}

	protected void setTargetPosition(Rotation2d position) {
		motor.applyRequest(positionRequest.withSetPoint(position));
	}

	public double getVoltage() {
		return voltageSignal.getLatestValue();
	}

	protected void setVoltage(double voltage) {
		motor.applyRequest(voltageRequest.withSetPoint(voltage));
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	protected void setPower(double power) {
		motor.setPower(power);
	}

	protected void stayInPlace() {
		setTargetPosition(positionSignal.getLatestValue());
	}

	public boolean isAtPosition(Rotation2d position, Rotation2d tolerance) {
		return ToleranceMath.isNearWrapped(position, positionSignal.getLatestValue(), tolerance);
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
		motor.updateSimulation();
	}

	private void updateInputs() {
		motor.updateInputs(positionSignal, voltageSignal);
	}

}
