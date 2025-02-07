package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.joysticks.Axis;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IAngleEncoder;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.arm.factory.KrakenX60ArmBuilder;
import frc.utils.battery.BatteryUtil;
import frc.utils.calibration.sysid.SysIdCalibrator;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

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
	private Rotation2d minSoftLimit;

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
		this.sysIdCalibrator = new SysIdCalibrator(motor.getSysidConfigInfo(), this, (voltage) -> setVoltage(voltage + getKgVoltage()));
		this.minSoftLimit = ArmConstants.ELEVATOR_CLOSED_REVERSED_SOFTWARE_LIMIT;

		periodic();
		resetByEncoderPosition();
		setDefaultCommand(getCommandsBuilder().stayInPlace());
	}

	public ArmCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public Rotation2d getPosition() {
		return motorPositionSignal.getLatestValue();
	}

	private double getKgVoltage() {
		return Robot.ROBOT_TYPE.isReal() ? KrakenX60ArmBuilder.kG * getPosition().getCos() : 0;
	}

	@Override
	protected void subsystemPeriodic() {
		motor.updateSimulation();
		updateInputs();
		Logger.recordOutput(getLogPath() + "/MinLimit", minSoftLimit);
	}

	private void updateInputs() {
		motor.updateInputs(motorPositionSignal, motorVoltageSignal);
		encoder.updateInputs(encoderPositionSignal);
	}

	public void setMinSoftLimit(Rotation2d minSoftLimit) {
		this.minSoftLimit = minSoftLimit;
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
		Logger.recordOutput(getLogPath() + "/TargetPose", position);
		if (minSoftLimit.getDegrees() > position.getDegrees()) {
			Logger.recordOutput(getLogPath() + "/TargetPoseUnderLimit", true);
			stayInPlace(); //todo fix
		} else {
			Logger.recordOutput(getLogPath() + "/TargetPoseUnderLimit", false);
			motor.applyRequest(positionRequest.withSetPoint(position));
		}
	}

	protected void stayInPlace() {
		Rotation2d target = motorPositionSignal.getLatestValue();
		if (target.getDegrees() < minSoftLimit.getDegrees()){
			target = minSoftLimit;
		} else if (target.getDegrees() > ArmConstants.FORWARD_SOFTWARE_LIMIT.getDegrees()) {
			target = ArmConstants.FORWARD_SOFTWARE_LIMIT;
		}
		setTargetPosition(target);
	}

	public boolean isAtPosition(Rotation2d position, Rotation2d tolerance) {
		return motorPositionSignal.isNear(position, tolerance);
	}

	public void applyCalibrationBindings(SmartJoystick joystick) {
		// Calibrate kG using phoenix tuner by setting the voltage

		// Check limits
		joystick.R1.whileTrue(
			commandsBuilder.setPower(
				() -> joystick.getAxisValue(Axis.LEFT_Y) * ArmConstants.CALIBRATION_MAX_POWER
					+ (getKgVoltage() / BatteryUtil.getCurrentVoltage())
			)
		);

		// Calibrate feed forward using sys id:
		sysIdCalibrator.setAllButtonsForCalibration(joystick);

		// Calibrate PID using phoenix tuner and these bindings:
		joystick.POV_UP.onTrue(commandsBuilder.moveToPosition(ArmState.L4.getPosition()));
		joystick.POV_DOWN.onTrue(commandsBuilder.moveToPosition(ArmState.INTAKE.getPosition()));
		joystick.POV_LEFT.onTrue(commandsBuilder.moveToPosition(ArmState.L1.getPosition()));
		joystick.POV_RIGHT.onTrue(commandsBuilder.moveToPosition(ArmState.L2.getPosition()));

		// Calibrate max acceleration and cruise velocity by the equations: max acceleration = (12 + Ks)/2kA, cruise velocity =(12 + Ks)/kV
	}

}
