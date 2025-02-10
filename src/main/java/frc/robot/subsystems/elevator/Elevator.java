package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.joysticks.Axis;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.elevator.factory.KrakenX60ElevatorBuilder;
import frc.robot.subsystems.elevator.records.ElevatorMotorSignals;
import frc.utils.Conversions;
import frc.utils.battery.BatteryUtil;
import frc.utils.calibration.sysid.SysIdCalibrator;
import org.littletonrobotics.junction.Logger;

public class Elevator extends GBSubsystem {

	private static double MAX_CALIBRATION_POWER = 0.1;

	private final ControllableMotor rightMotor;
	private final ElevatorMotorSignals rightMotorSignals;

	private final ControllableMotor leftMotor;
	private final ElevatorMotorSignals leftMotorSignals;

	private final IRequest<Rotation2d> positionRequest;
	private final IRequest<Double> voltageRequest;

	private final IDigitalInput limitSwitch;
	private final DigitalInputInputsAutoLogged digitalInputInputs;

	private final ElevatorCommandsBuilder commandsBuilder;
	private final SysIdCalibrator sysIdCalibrator;

	private boolean hasBeenResetBySwitch;
	private double ffCalibrationVoltage;
	private double currentTargetPositionMeters;

	public Elevator(
		String logPath,
		ControllableMotor rightMotor,
		ElevatorMotorSignals rightMotorSignals,
		ControllableMotor leftMotor,
		ElevatorMotorSignals leftMotorSignals,
		IRequest<Rotation2d> positionRequest,
		IRequest<Double> voltageRequest,
		IDigitalInput limitSwitch
	) {
		super(logPath);

		this.rightMotor = rightMotor;
		this.rightMotorSignals = rightMotorSignals;

		this.leftMotor = leftMotor;
		this.leftMotorSignals = leftMotorSignals;

		this.positionRequest = positionRequest;
		this.voltageRequest = voltageRequest;
		this.limitSwitch = limitSwitch;
		this.digitalInputInputs = new DigitalInputInputsAutoLogged();
		hasBeenResetBySwitch = false;
		this.ffCalibrationVoltage = 0;

		this.commandsBuilder = new ElevatorCommandsBuilder(this);
		this.sysIdCalibrator = new SysIdCalibrator(rightMotor.getSysidConfigInfo(), this, (voltage) -> setVoltage(voltage + getKgVoltage()));

		resetMotors(ElevatorConstants.MINIMUM_HEIGHT_METERS);
		periodic();
		setDefaultCommand(getCommandsBuilder().stayInPlace());
	}

	public double getKgVoltage() {
		return Robot.ROBOT_TYPE.isReal() ? KrakenX60ElevatorBuilder.kG : 0;
	}

	public ElevatorCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public SysIdCalibrator getSysIdCalibrator() {
		return sysIdCalibrator;
	}

	public double getElevatorPositionMeters() {
		return convertRotationsToMeters(rightMotorSignals.positionSignal().getLatestValue());
	}

	public boolean hasBeenResetBySwitch() {
		return hasBeenResetBySwitch;
	}

	public boolean isAtBackwardsLimit() {
		return digitalInputInputs.debouncedValue;
	}

	@Override
	protected void subsystemPeriodic() {
		// Update Simulation checks if ROBOT_TYPE.isSimulation() inside the function and acts accordingly.
		rightMotor.updateSimulation();
		leftMotor.updateSimulation();
		updateInputs();
		if (handleReset()) {
			updateInputs();
		}
		log();
	}

	private void updateInputs() {
		rightMotor.updateInputs(rightMotorSignals.positionSignal(), rightMotorSignals.voltageSignal());
		rightMotor.updateInputs(rightMotorSignals.otherSignals());

		leftMotor.updateInputs(leftMotorSignals.positionSignal(), leftMotorSignals.voltageSignal());
		leftMotor.updateInputs(leftMotorSignals.otherSignals());

		limitSwitch.updateInputs(digitalInputInputs);
		Logger.processInputs(getLogPath() + "/LimitSwitch", digitalInputInputs);
	}

	private void log() {
		Logger.recordOutput(getLogPath() + "/PositionMeters", getElevatorPositionMeters());
		Logger.recordOutput(getLogPath() + "/IsAtBackwardsLimit", isAtBackwardsLimit());
		Logger.recordOutput(getLogPath() + "/HasBeenResetBySwitch", hasBeenResetBySwitch);
		Logger.recordOutput(getLogPath() + "/FFCalibrationVoltage", ffCalibrationVoltage);
		Logger.recordOutput(getLogPath() + "/TargetPositionMeters", currentTargetPositionMeters);
	}

	public void resetMotors(double positionMeters) {
		Rotation2d convertedPosition = convertMetersToRotations(positionMeters);
		rightMotor.resetPosition(convertedPosition);
		leftMotor.resetPosition(convertedPosition);
	}

	public void setBrake(boolean brake) {
		rightMotor.setBrake(brake);
		leftMotor.setBrake(brake);
	}

	protected void stop() {
		rightMotor.stop();
		leftMotor.stop();
	}

	protected void setPower(double power) {
		rightMotor.setPower(power);
		leftMotor.setPower(power);
	}

	protected void setVoltage(double voltage) {
		rightMotor.applyRequest(voltageRequest.withSetPoint(voltage));
		leftMotor.applyRequest(voltageRequest.withSetPoint(voltage));
	}

	protected void setTargetPositionMeters(double targetPositionMeters) {
		currentTargetPositionMeters = targetPositionMeters;
		Rotation2d targetPosition = convertMetersToRotations(targetPositionMeters);
		rightMotor.applyRequest(positionRequest.withSetPoint(targetPosition));
		leftMotor.applyRequest(positionRequest.withSetPoint(targetPosition));
	}

	protected void stayInPlace() {
		setTargetPositionMeters(getElevatorPositionMeters());
	}

	public boolean isAtPosition(double positionMeters, double toleranceMeters) {
		return MathUtil.isNear(positionMeters, getElevatorPositionMeters(), toleranceMeters);
	}

	private boolean shouldResetByMinimumPosition() {
		return getElevatorPositionMeters() <= ElevatorConstants.MINIMUM_HEIGHT_METERS;
	}

	private boolean shouldResetByLimitSwitch() {
		return isAtBackwardsLimit();
	}

	private boolean handleReset() {
		if (DriverStation.isEnabled() || hasBeenResetBySwitch()) {
			return false;
		}
		if (shouldResetByLimitSwitch()) {
			hasBeenResetBySwitch = true;
		}
		if (shouldResetByMinimumPosition() || shouldResetByLimitSwitch()) {
			resetMotors(ElevatorConstants.MINIMUM_HEIGHT_METERS);
			return true;
		}
		return false;
	}

	public void applyCalibrationBindings(SmartJoystick joystick) {
		joystick.getAxisAsButton(Axis.LEFT_TRIGGER).whileTrue(commandsBuilder.setVoltage(() -> ffCalibrationVoltage));
		joystick.R1.onTrue(new InstantCommand(() -> ffCalibrationVoltage = ffCalibrationVoltage + 0.01));
		joystick.L1.onTrue(new InstantCommand(() -> ffCalibrationVoltage = ffCalibrationVoltage - 0.01));

		joystick.getAxisAsButton(Axis.RIGHT_TRIGGER)
			.whileTrue(
				commandsBuilder.setPower(
					() -> joystick.getAxisValue(Axis.LEFT_Y) * MAX_CALIBRATION_POWER + (getKgVoltage() / BatteryUtil.getCurrentVoltage())
				)
			);

		// The sysid outputs will be logged to the "CTRE Signal Logger". Use phoenix tuner x to extract the position, velocity, motorVoltage,
		// state signals into wpilog. Then enter the wpilog into wpilib sysid app and make sure you enter all info in the correct places. (see
		// wpilib sysid in google)
		sysIdCalibrator.setAllButtonsForCalibration(joystick);

		// PID Testing
		joystick.POV_DOWN.onTrue(commandsBuilder.setTargetPositionMeters(ElevatorState.L1.getHeightMeters()));
		joystick.POV_LEFT.onTrue(commandsBuilder.setTargetPositionMeters(ElevatorState.L2.getHeightMeters()));
		joystick.POV_RIGHT.onTrue(commandsBuilder.setTargetPositionMeters(ElevatorState.L3.getHeightMeters()));
		joystick.POV_UP.onTrue(commandsBuilder.setTargetPositionMeters(ElevatorState.L4.getHeightMeters()));

		// Calibrate max acceleration and cruse velocity by the equations: max acceleration = (12 + Ks)/2kA cruise velocity = (12 + Ks)/kV
	}

	public static double convertRotationsToMeters(Rotation2d position) {
		return Conversions.angleToDistance(position, ElevatorConstants.DRUM_DIAMETER_METERS);
	}

	public static Rotation2d convertMetersToRotations(double meters) {
		return Conversions.distanceToAngle(meters, ElevatorConstants.DRUM_DIAMETER_METERS);
	}

}
