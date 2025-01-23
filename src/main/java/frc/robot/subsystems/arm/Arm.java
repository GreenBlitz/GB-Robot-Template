package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IAngleEncoder;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;

public class Arm extends GBSubsystem {

	private final ControllableMotor motor;
	private final IRequest<Rotation2d> positionRequest;
	private final IRequest<Double> voltageRequest;
	private final InputSignal<Rotation2d> motorPositionSignal;
	private final InputSignal<Double> voltageSignal;
	private final IAngleEncoder encoder;
	private final InputSignal<Rotation2d> encoderPositionSignal;
	private final ArmCommandsBuilder commandsBuilder;

	public Arm(
		String logPath,
		ControllableMotor motor,
		IRequest<Rotation2d> positionRequest,
		IRequest<Double> voltageRequest,
		InputSignal<Rotation2d> motorPositionSignal,
		InputSignal<Double> voltageSignal,
		IAngleEncoder encoder,
		InputSignal<Rotation2d> encoderPositionSignal
	) {
		super(logPath);
		this.motor = motor;
		this.positionRequest = positionRequest;
		this.voltageRequest = voltageRequest;
		this.motorPositionSignal = motorPositionSignal;
		this.voltageSignal = voltageSignal;
		this.encoder = encoder;
		this.encoderPositionSignal = encoderPositionSignal;
		this.commandsBuilder = new ArmCommandsBuilder(this);

		periodic();
		setDefaultCommand(getCommandsBuilder().stayInPlace());
	}

	public ArmCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	@Override
	protected void subsystemPeriodic() {
		motor.updateSimulation();
		updateInputs();
	}

	private void updateInputs() {
		motor.updateInputs(motorPositionSignal, voltageSignal);
		encoder.updateInputs(encoderPositionSignal);
	}

	protected void resetByEncoderPosition() {
		motor.resetPosition(encoderPositionSignal.getLatestValue());
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	protected void stop() {
		motor.stop();
	}

	protected void setVoltage(double voltage) {
		motor.applyRequest(voltageRequest.withSetPoint(voltage));
	}

	protected void setPower(double power) {
		motor.setPower(power);
	}

	protected void setTargetPosition(Rotation2d position) {
		motor.applyRequest(positionRequest.withSetPoint(position));
	}

	protected void stayInPlace() {
		setTargetPosition(motorPositionSignal.getLatestValue());
	}

	public boolean isAtPosition(Rotation2d position, double tolerance) {
		return MathUtil.isNear(position.getDegrees(), motorPositionSignal.getLatestValue().getDegrees(), tolerance);
	}

}
