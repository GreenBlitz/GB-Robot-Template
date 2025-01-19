package frc.robot.subsystems.elevator;

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
import frc.utils.math.ToleranceMath;
import org.littletonrobotics.junction.Logger;

public class Elevator extends GBSubsystem {

	private final DigitalInputInputsAutoLogged digitalInputInputs;

	private final ControllableMotor firstMotor;
	private final ElevatorMotorSignals firstMotorSignals;

	private final ControllableMotor secondMotor;
	private final ElevatorMotorSignals secondMotorSignals;

	private final IRequest<Rotation2d> positionRequest;
	private final IRequest<Double> voltageRequest;
	private final IDigitalInput limitSwitch;
	private final ElevatorCommandsBuilder commandsBuilder;

	private final SysIdCalibrator firstMotorSysIdCalibrator;
	private final SysIdCalibrator secondMotorSysIdCalibrator;

	private boolean hasBeenResetBySwitch;

	public Elevator(
		String logPath,
		ControllableMotor firstMotor,
		ElevatorMotorSignals firstMotorSignals,
		ControllableMotor secondMotor,
		ElevatorMotorSignals secondMotorSignals,
		IRequest<Rotation2d> positionRequest,
		IRequest<Double> voltageRequest,
		IDigitalInput limitSwitch
	) {
		super(logPath);

		this.firstMotor = firstMotor;
		this.firstMotorSignals = firstMotorSignals;

		this.secondMotor = secondMotor;
		this.secondMotorSignals = secondMotorSignals;

		this.positionRequest = positionRequest;
		this.voltageRequest = voltageRequest;
		this.limitSwitch = limitSwitch;
		this.digitalInputInputs = new DigitalInputInputsAutoLogged();
		hasBeenResetBySwitch = false;
		this.commandsBuilder = new ElevatorCommandsBuilder(this);

		this.firstMotorSysIdCalibrator = new SysIdCalibrator(
			new SysIdCalibrator.SysIdConfigInfo(firstMotor.getSysidConfigInfo().config(), true),
			this,
			this::setVoltage
		);
		this.secondMotorSysIdCalibrator = new SysIdCalibrator(
			new SysIdCalibrator.SysIdConfigInfo(secondMotor.getSysidConfigInfo().config(), true),
			this,
			this::setVoltage
		);

		updateInputs();
	}

	public ElevatorCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public SysIdCalibrator getFirstMotorSysIdCalibrator() {
		return firstMotorSysIdCalibrator;
	}

	public SysIdCalibrator getSecondMotorSysIdCalibrator() {
		return secondMotorSysIdCalibrator;
	}

	public double getElevatorPositionMeters() {
		return convertRotationsToMeters(firstMotorSignals.positionSignal().getLatestValue());
	}

	public boolean hasBeenResetBySwitch() {
		return hasBeenResetBySwitch;
	}

	public boolean isAtBackwardsLimit() {
		return digitalInputInputs.debouncedValue;
	}

	@Override
	protected void subsystemPeriodic() {
		firstMotor.updateSimulation();
		secondMotor.updateSimulation();
		updateInputs();
		if (handleReset()) {
			updateInputs();
		}
		log();
	}

	private void updateInputs() {
		firstMotor.updateInputs(firstMotorSignals.positionSignal(), firstMotorSignals.voltageSignal());
		firstMotor.updateInputs(firstMotorSignals.otherSignals());

		secondMotor.updateInputs(secondMotorSignals.positionSignal(), secondMotorSignals.voltageSignal());
		secondMotor.updateInputs(secondMotorSignals.otherSignals());

		limitSwitch.updateInputs(digitalInputInputs);
		Logger.processInputs(getLogPath() + "LimitSwitch/", digitalInputInputs);
	}
	
	private void log(){
		Logger.recordOutput(getLogPath() + "PositionMeters", getElevatorPositionMeters());
		Logger.recordOutput(getLogPath() + "isAtBackwardsLimit", isAtBackwardsLimit());
		Logger.recordOutput(getLogPath() + "hasBeenResetBySwitch", hasBeenResetBySwitch);
	}

	public void resetMotors(double positionMeters) {
		Rotation2d convertedPosition = convertMetersToRotations(positionMeters);
		firstMotor.resetPosition(convertedPosition);
		secondMotor.resetPosition(convertedPosition);
	}

	public void setBrake(boolean brake) {
		firstMotor.setBrake(brake);
		secondMotor.setBrake(brake);
	}

	protected void stop() {
		firstMotor.stop();
		secondMotor.stop();
	}

	protected void setPower(double power) {
		firstMotor.setPower(power);
		secondMotor.setPower(power);
	}

	protected void setVoltage(double voltage) {
		firstMotor.applyRequest(voltageRequest.withSetPoint(voltage));
		secondMotor.applyRequest(voltageRequest.withSetPoint(voltage));
	}

	protected void setTargetPositionMeters(double targetPositionMeters) {
		Rotation2d targetPosition = convertMetersToRotations(targetPositionMeters);
		firstMotor.applyRequest(positionRequest.withSetPoint(targetPosition));
		secondMotor.applyRequest(positionRequest.withSetPoint(targetPosition));
	}

	protected void stayInPlace() {
		setTargetPositionMeters(getElevatorPositionMeters());
	}

	public boolean isAtPosition(double positionMeters, double toleranceMeters) {
		return ToleranceMath.isNearWrapped(
			convertMetersToRotations(positionMeters),
			convertMetersToRotations(getElevatorPositionMeters()),
			convertMetersToRotations(toleranceMeters)
		);
	}

	private boolean shouldResetByMinimumPosition() {
		return getElevatorPositionMeters() <= ElevatorConstants.MINIMUM_HEIGHT_METERS;
	}

	private boolean shouldResetByLimitSwitch() {
		return isAtBackwardsLimit() && DriverStation.isDisabled() && !hasBeenResetBySwitch;
	}

	private boolean handleReset() {
		if (shouldResetByMinimumPosition() || shouldResetByLimitSwitch()) {
			resetMotors(ElevatorConstants.MINIMUM_HEIGHT_METERS);
			return true;
		}
		return false;
	}

	public static double convertRotationsToMeters(Rotation2d position) {
		return Conversions.angleToDistance(position, ElevatorConstants.DRUM_RADIUS);
	}

	public static Rotation2d convertMetersToRotations(double meters) {
		return Conversions.distanceToAngle(meters, ElevatorConstants.DRUM_RADIUS);
	}

}
