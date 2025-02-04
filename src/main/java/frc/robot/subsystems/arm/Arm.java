package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IAngleEncoder;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.calibration.sysid.SysIdCalibrator;
import org.littletonrobotics.junction.Logger;

public class Arm extends GBSubsystem {

	private final ControllableMotor motor;
	private final IRequest<Rotation2d> positionRequest;
	private final IRequest<Double> voltageRequest;
	private final InputSignal<Rotation2d> motorPositionSignal;
	private final InputSignal<Double> motorVoltageSignal;
	private final IAngleEncoder encoder;
	private final InputSignal<Rotation2d> encoderPositionSignal;
	private final ArmCommandsBuilder commandsBuilder;
	private final SysIdCalibrator sysIdCalibrator;
	private Rotation2d targetPosition;

	public Arm(
		String logPath,
		ControllableMotor motor,
		IRequest<Rotation2d> positionRequest,
		IRequest<Double> voltageRequest,
		InputSignal<Rotation2d> motorPositionSignal,
		InputSignal<Double> motorVoltageSignal,
		IAngleEncoder encoder,
		InputSignal<Rotation2d> encoderPositionSignal
	) {
		super(logPath);
		this.motor = motor;
		this.positionRequest = positionRequest;
		this.voltageRequest = voltageRequest;
		this.motorPositionSignal = motorPositionSignal;
		this.motorVoltageSignal = motorVoltageSignal;
		this.encoder = encoder;
		this.encoderPositionSignal = encoderPositionSignal;
		this.commandsBuilder = new ArmCommandsBuilder(this);
		this.sysIdCalibrator = new SysIdCalibrator(motor.getSysidConfigInfo(), this, this::setVoltage);
		this.targetPosition = getPosition();

		periodic();
		setDefaultCommand(getCommandsBuilder().stayInPlace());
	}

	public ArmCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public Rotation2d getPosition() {
		return motorPositionSignal.getLatestValue();
	}

	public SysIdCalibrator getSysIdCalibrator() {
		return sysIdCalibrator;
	}

	@Override
	protected void subsystemPeriodic() {
		motor.updateSimulation();
		Logger.recordOutput(getLogPath() + "/Target position", targetPosition);
		updateInputs();
	}

	private void updateInputs() {
		motor.updateInputs(motorPositionSignal, motorVoltageSignal);
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

	protected void setPower(double power) {
		motor.setPower(power);
	}

	protected void setVoltage(double voltage) {
		motor.applyRequest(voltageRequest.withSetPoint(voltage));
	}

	protected void setTargetPosition(Rotation2d position) {
		motor.applyRequest(positionRequest.withSetPoint(position));
		this.targetPosition = position;
	}

	protected void stayInPlace() {
		setTargetPosition(motorPositionSignal.getLatestValue());
	}

	public boolean isAtPosition(Rotation2d position, Rotation2d tolerance) {
		return motorPositionSignal.isNear(position, tolerance);
	}

}
