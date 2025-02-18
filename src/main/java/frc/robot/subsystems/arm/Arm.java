package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
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
import frc.utils.alerts.Alert;
import frc.utils.battery.BatteryUtil;
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
	private Rotation2d reversedSoftLimit;

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
		this.reversedSoftLimit = ArmConstants.ELEVATOR_CLOSED_REVERSED_SOFTWARE_LIMIT;

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
		log();
	}

	private void updateInputs() {
		motor.updateInputs(motorPositionSignal, motorVoltageSignal);
		encoder.updateInputs(encoderPositionSignal);
	}

	private void log() {
		Logger.recordOutput(getLogPath() + "/ReversedSoftLimit", reversedSoftLimit);
		Logger.recordOutput(getLogPath() + "/TargetPose", positionRequest.getSetPoint());
	}

	public void setReversedSoftLimit(Rotation2d reversedSoftLimit) {
		this.reversedSoftLimit = reversedSoftLimit;
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

	protected void setTargetPosition(Rotation2d targetPosition) {
		if (reversedSoftLimit.getDegrees() <= targetPosition.getDegrees()) {
			motor.applyRequest(positionRequest.withSetPoint(targetPosition));
		} else {
			new Alert(Alert.AlertType.WARNING, getLogPath() + "/TargetPoseUnderLimit").report();
			stayInPlace();
		}
	}

	protected void stayInPlace() {
		Rotation2d limitedPosition = Rotation2d.fromDegrees(
			MathUtil.clamp(
				motorPositionSignal.getLatestValue().getDegrees(),
				reversedSoftLimit.getDegrees(),
				ArmConstants.FORWARD_SOFTWARE_LIMIT.getDegrees()
			)
		);
		setTargetPosition(limitedPosition);
	}

	public boolean isAtPosition(Rotation2d position, Rotation2d tolerance) {
		return motorPositionSignal.isNear(position, tolerance);
	}

	public boolean isPastPosition(Rotation2d position) {
		return motorPositionSignal.isGreater(position);
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
