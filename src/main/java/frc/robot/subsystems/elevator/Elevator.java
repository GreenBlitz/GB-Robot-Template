package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.elevator.records.ElevatorMotorStuff;
import frc.utils.Conversions;
import org.littletonrobotics.junction.Logger;

public class Elevator extends GBSubsystem {

	private final String limitSwitchLogPath;
	private final DigitalInputInputsAutoLogged digitalInputInputsAutoLogged;
	private final ElevatorMotorStuff firstMotorStuff;
	private final ElevatorMotorStuff secondMotorStuff;
	private final ControllableMotor firstMotor;
	private final ControllableMotor secondMotor;
	private final IDigitalInput limitSwitch;
	private final ElevatorCommandsBuilder commandsBuilder;

	private boolean hasBeenResetBySwitch;

	public Elevator(
		String logPath,
		String limitSwitchLogPath,
		ElevatorMotorStuff firstMotorStuff,
		ElevatorMotorStuff secondMotorStuff,
		IDigitalInput limitSwitch
	) {
		super(logPath);
		this.limitSwitchLogPath = limitSwitchLogPath;

		this.firstMotorStuff = firstMotorStuff;
		this.firstMotor = firstMotorStuff.motor();

		this.secondMotorStuff = secondMotorStuff;
		this.secondMotor = secondMotorStuff.motor();

		this.limitSwitch = limitSwitch;
		this.digitalInputInputsAutoLogged = new DigitalInputInputsAutoLogged();
		hasBeenResetBySwitch = false;
		this.commandsBuilder = new ElevatorCommandsBuilder(this);

		updateInputs();
	}

	public ElevatorCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public double getElevatorPositionMeters() {
		return convertRotationsToMeters(firstMotorStuff.signals().positionSignal().getLatestValue());
	}

	public boolean hasBeenResetBySwitch() {
		return hasBeenResetBySwitch;
	}

	public boolean isAtBackwardsLimit() {
		return digitalInputInputsAutoLogged.debouncedValue;
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
		handleReset();
		log();
	}

	private void updateInputs() {
		firstMotor.updateInputs(firstMotorStuff.signals().positionSignal(), firstMotorStuff.signals().voltageSignal());
		firstMotor.updateInputs(firstMotorStuff.signals().otherSignals());

		secondMotor.updateInputs(secondMotorStuff.signals().positionSignal(), firstMotorStuff.signals().voltageSignal());
		secondMotor.updateInputs(secondMotorStuff.signals().otherSignals());

		limitSwitch.updateInputs(digitalInputInputsAutoLogged);
	}

	private void log() {
		Logger.recordOutput(getLogPath() + "PositionMeters", getElevatorPositionMeters());
		Logger.recordOutput(getLogPath() + "isAtBackwardsLimit", isAtBackwardsLimit());
		Logger.recordOutput(getLogPath() + "hasBeenResetBySwitch", hasBeenResetBySwitch);
		Logger.processInputs(limitSwitchLogPath, digitalInputInputsAutoLogged);
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
		firstMotor.applyRequest(firstMotorStuff.requests().voltageRequest().withSetPoint(voltage));
		secondMotor.applyRequest(secondMotorStuff.requests().voltageRequest().withSetPoint(voltage));
	}

	protected void setTargetPositionMeters(double targetPositionMeters) {
		Rotation2d targetPosition = convertMetersToRotations(targetPositionMeters);
		firstMotor.applyRequest(firstMotorStuff.requests().positionRequest().withSetPoint(targetPosition));
		secondMotor.applyRequest(secondMotorStuff.requests().positionRequest().withSetPoint(targetPosition));
	}

	protected void stayInPlace(){
		stop();
	}

	private void dynamicReset() {
		if (getElevatorPositionMeters() <= ElevatorConstants.MINIMUM_ACHIEVABLE_POSITION_METERS) {
			resetMotors(ElevatorConstants.MINIMUM_ACHIEVABLE_POSITION_METERS);
		}
	}

	private void limitSwitchReset() {
		if (isAtBackwardsLimit() && DriverStation.isDisabled() && !hasBeenResetBySwitch) {
			resetMotors(ElevatorConstants.MINIMUM_ACHIEVABLE_POSITION_METERS);
			hasBeenResetBySwitch = true;
		}
	}

	private void handleReset() {
		dynamicReset();
		limitSwitchReset();
	}

	private double convertRotationsToMeters(Rotation2d position) {
		return Conversions.angleToDistance(position, ElevatorConstants.DRUM_RADIUS);
	}

	private Rotation2d convertMetersToRotations(double meters) {
		return Conversions.distanceToAngle(meters, ElevatorConstants.DRUM_RADIUS);
	}

}
