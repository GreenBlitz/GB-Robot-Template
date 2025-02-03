package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.elevator.records.ElevatorMotorSignals;
import frc.utils.Conversions;
import frc.utils.calibration.sysid.SysIdCalibrator;
import org.littletonrobotics.junction.Logger;

public class Elevator extends GBSubsystem {

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

		this.commandsBuilder = new ElevatorCommandsBuilder(this);
		this.sysIdCalibrator = new SysIdCalibrator(rightMotor.getSysidConfigInfo(), this, this::setVoltage);

		periodic();

		setDefaultCommand(getCommandsBuilder().stayInPlace());
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
		if (DriverStation.isDisabled() || !hasBeenResetBySwitch()) {
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

	public static double convertRotationsToMeters(Rotation2d position) {
		return Conversions.angleToDistance(position, ElevatorConstants.DRUM_DIAMETER_METERS);
	}

	public static Rotation2d convertMetersToRotations(double meters) {
		return Conversions.distanceToAngle(meters, ElevatorConstants.DRUM_DIAMETER_METERS);
	}

}
