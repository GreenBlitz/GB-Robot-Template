package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class Elevator extends GBSubsystem {

	private final DigitalInputInputsAutoLogged digitalInputsInputs;
	private final ElevatorCommandsBuilder commandsBuilder;
	private final IRequest<Rotation2d> positionRequest;
	private final IRequest<Double> voltageRequest;

	private final ElevatorStuff elevatorStuff;
	private final ControllableMotor frontMotor;
	private final IDigitalInput limitSwitch;

	public Elevator(ElevatorStuff elevatorStuff) {
		super(elevatorStuff.logPath());

		this.frontMotor = elevatorStuff.motorStuff().motor();
		this.limitSwitch = elevatorStuff.digitalInput();
		this.digitalInputsInputs = new DigitalInputInputsAutoLogged();
		this.elevatorStuff = elevatorStuff;
		this.positionRequest = elevatorStuff.positionRequest();
		this.voltageRequest = elevatorStuff.voltageRequest();
		this.commandsBuilder = new ElevatorCommandsBuilder(this);

		frontMotor.resetPosition(metersToMotorRotations(ElevatorConstants.MINIMUM_ACHIEVABLE_POSITION_METERS));
		updateInputs();
	}

	public ElevatorCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	protected void setPower(double power) {
		frontMotor.setPower(power);
	}

	protected void setVoltage(double voltage) {
		frontMotor.applyDoubleRequest(voltageRequest.withSetPoint(voltage));
	}

	protected void stop() {
		frontMotor.stop();
	}

	public void setBrake(boolean brake) {
		frontMotor.setBrake(brake);
	}

	protected void setTargetPositionMeters(double position) {
		frontMotor.applyAngleRequest(positionRequest.withSetPoint(metersToMotorRotations(position)));
	}

	public boolean isAtBackwardLimit() {
		return digitalInputsInputs.debouncedValue;
	}

	public double getPositionMeters() {
		return motorRotationsToMeters(elevatorStuff.motorStuff().positionSignal().getLatestValue());
	}

	protected void stayInPlace() {
		setTargetPositionMeters(getPositionMeters());
	}

	protected void updateInputs() {
		limitSwitch.updateInputs(digitalInputsInputs);
		frontMotor.updateSignals(elevatorStuff.motorStuff().positionSignal(), elevatorStuff.motorStuff().voltageSignal());

		Logger.processInputs(elevatorStuff.digitalInputsLogPath(), digitalInputsInputs);
		Logger.recordOutput(getLogPath() + "isAtBackwardLimit", isAtBackwardLimit());
		Logger.recordOutput(getLogPath() + "elevatorPosition", getPositionMeters());
	}

	@Override
	protected void subsystemPeriodic() {
		if (ElevatorConstants.MINIMUM_ACHIEVABLE_POSITION_METERS > getPositionMeters()) {
			frontMotor.resetPosition(metersToMotorRotations(ElevatorConstants.MINIMUM_ACHIEVABLE_POSITION_METERS));
		}

		updateInputs();
	}

	public static double motorRotationsToMeters(Rotation2d rotations) {
		return rotations.getRotations() * ElevatorConstants.TIMING_PULLEY_ROTATIONS_TO_METERS_CONVERSION_RATIO;
	}

	public static Rotation2d metersToMotorRotations(double meters) {
		return Rotation2d.fromRotations(meters / ElevatorConstants.TIMING_PULLEY_ROTATIONS_TO_METERS_CONVERSION_RATIO);
	}

}
