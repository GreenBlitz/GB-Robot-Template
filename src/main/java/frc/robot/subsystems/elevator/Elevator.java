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
import frc.robot.hardware.interfaces.IDynamicMotionMagicRequest;
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

	private final ControllableMotor motor;
	private final ElevatorMotorSignals signals;

	private final IDynamicMotionMagicRequest positionRequest;
	private final IRequest<Double> voltageRequest;

	private final IDigitalInput limitSwitch;
	private final DigitalInputInputsAutoLogged digitalInputInputs;

	private final ElevatorCommandsBuilder commandsBuilder;
	private final SysIdCalibrator sysIdCalibrator;
	private final ElevatorInputsAutoLogged inputs;

	private boolean hasBeenResetBySwitch;
	private double ffCalibrationVoltage;
	private double targetPositionMeters;

	public Elevator(
		String logPath,
		ControllableMotor motor,
		ElevatorMotorSignals signals,
		IDynamicMotionMagicRequest positionRequest,
		IRequest<Double> voltageRequest,
		IDigitalInput limitSwitch
	) {
		super(logPath);

		this.motor = motor;
		this.signals = signals;
		this.inputs = new ElevatorInputsAutoLogged();

		this.positionRequest = positionRequest;
		this.voltageRequest = voltageRequest;
		this.limitSwitch = limitSwitch;
		this.digitalInputInputs = new DigitalInputInputsAutoLogged();
		hasBeenResetBySwitch = false;
		this.ffCalibrationVoltage = 0;

		this.commandsBuilder = new ElevatorCommandsBuilder(this);
		this.sysIdCalibrator = new SysIdCalibrator(motor.getSysidConfigInfo(), this, (voltage) -> setVoltage(voltage + getKgVoltage()));

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
		return convertRotationsToMeters(signals.positionSignal().getLatestValue());
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
		motor.updateSimulation();
		updateInputs();
		if (handleReset()) {
			updateInputs();
		}
//		log();
	}

	private void updateInputs() {
		motor.updateInputs();

		inputs.data = new ElevatorInputs.ElevatorData(
			signals.positionSignal().getAndUpdateValue().getRadians(),
			signals.voltageSignal().getAndUpdateValue(),
			getElevatorPositionMeters(),
			targetPositionMeters
		);
		Logger.processInputs(getLogPath(), inputs);

//		limitSwitch.updateInputs(digitalInputInputs);
//		Logger.processInputs(getLogPath() + "/LimitSwitch", digitalInputInputs);
	}

	private void log() {
		Logger.recordOutput(getLogPath() + "/IsAtBackwardsLimit", isAtBackwardsLimit());
		Logger.recordOutput(getLogPath() + "/HasBeenResetBySwitch", hasBeenResetBySwitch);
		Logger.recordOutput(getLogPath() + "/FFCalibrationVoltage", ffCalibrationVoltage);
	}

	public void resetMotors(double positionMeters) {
		Rotation2d convertedPosition = convertMetersToRotations(positionMeters);
		motor.resetPosition(convertedPosition);
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

	protected void setTargetPositionMeters(double targetPositionMeters) {
		setTargetPositionMeters(
			targetPositionMeters,
			ElevatorConstants.CRUISE_VELOCITY_METERS_PER_SECOND,
			ElevatorConstants.ACCELERATION_METERS_PER_SECOND_SQUARED
		);
	}

	protected void setTargetPositionMeters(
		double targetPositionMeters,
		double maxVelocityMetersPerSecond,
		double maxAccelerationMetersPerSecondSquared
	) {
		this.targetPositionMeters = targetPositionMeters;
		Rotation2d targetPosition = convertMetersToRotations(targetPositionMeters);
		Rotation2d maxVelocityRotation2dPerSecond = convertMetersToRotations(maxVelocityMetersPerSecond);
		Rotation2d maxAccelerationRotation2dPerSecond = convertMetersToRotations(maxAccelerationMetersPerSecondSquared);

		motor.applyRequest(
			positionRequest.withSetPoint(targetPosition)
				.withMaxVelocityRotation2dPerSecond(maxVelocityRotation2dPerSecond)
				.withMaxAccelerationRotation2dPerSecondSquared(maxAccelerationRotation2dPerSecond)
		);
	}

	protected void stayInPlace() {
		setTargetPositionMeters(getElevatorPositionMeters());
	}

	public boolean isAtPosition(double positionMeters, double toleranceMeters) {
		return MathUtil.isNear(positionMeters, getElevatorPositionMeters(), toleranceMeters);
	}

	public boolean isPastPosition(double positionMeters) {
		return getElevatorPositionMeters() > positionMeters;
	}

	private boolean shouldResetByMinimumPosition() {
		return getElevatorPositionMeters() < ElevatorConstants.MINIMUM_HEIGHT_METERS;
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

		ElevatorStateHandler elevatorStateHandler = new ElevatorStateHandler(this);

		// PID Testing
		joystick.POV_DOWN.onTrue(elevatorStateHandler.setState(ElevatorState.CLOSED));
		joystick.POV_LEFT.onTrue(elevatorStateHandler.setState(ElevatorState.NET));
		joystick.POV_RIGHT.onTrue(elevatorStateHandler.setState(ElevatorState.PRE_L4));
		joystick.POV_UP.onTrue(elevatorStateHandler.setState(ElevatorState.L3));

		// Calibrate max acceleration and cruse velocity by the equations: max acceleration = (12 + Ks)/2kA cruise velocity = (12 + Ks)/kV
	}

	public static double convertRotationsToMeters(Rotation2d position) {
		return Conversions.angleToDistance(position, ElevatorConstants.DRUM_DIAMETER_METERS);
	}

	public static Rotation2d convertMetersToRotations(double meters) {
		return Conversions.distanceToAngle(meters, ElevatorConstants.DRUM_DIAMETER_METERS);
	}

}
