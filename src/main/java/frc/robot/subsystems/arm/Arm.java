package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
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

	private void updateInputs() {
		motor.updateInputs(positionSignal, voltageSignal);
	}

	@Override
	protected void subsystemPeriodic() {
		motor.updateSimulation();
		updateInputs();
	}

	protected void setTargetPosition(Rotation2d position) {
		motor.applyRequest(positionRequest.withSetPoint(position));
	}

	protected void setVoltage(double voltage) {
		motor.applyRequest(voltageRequest.withSetPoint(voltage));
	}

	protected void setPower(double power) {
		motor.setPower(power);
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	protected void stop(){
		motor.stop();
	}

	protected void stayInPlace() {
		setTargetPosition(positionSignal.getLatestValue());
	}

	public boolean isAtPosition(Rotation2d position, double tolerance) {
		return MathUtil.isNear(position.getDegrees(), positionSignal.getLatestValue().getDegrees(), tolerance);
	}

}
