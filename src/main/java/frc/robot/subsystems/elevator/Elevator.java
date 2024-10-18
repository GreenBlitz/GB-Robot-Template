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
	private final ControllableMotor motor;
	private final IDigitalInput limitSwitch;

	public Elevator(ElevatorStuff elevatorStuff) {
		super(elevatorStuff.logPath());

		this.motor = elevatorStuff.motorStuff().motor();
		this.limitSwitch = elevatorStuff.digitalInput();
		this.digitalInputsInputs = new DigitalInputInputsAutoLogged();
		this.elevatorStuff = elevatorStuff;
		this.positionRequest = elevatorStuff.positionRequest();
		this.voltageRequest = elevatorStuff.voltageRequest();

		motor.resetPosition(metersToMotorRotations(ElevatorConstants.MINIMUM_ACHIEVABLE_POSITION_METERS));

		this.commandsBuilder = new ElevatorCommandsBuilder(this);

		updateInputs();
	}

	public ElevatorCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	protected void setPower(double power) {
		motor.setPower(power);
	}

	protected void setVoltage(double voltage) {
		motor.applyDoubleRequest(voltageRequest.withSetPoint(voltage));
	}

	protected void stop() {
		motor.stop();
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	protected void setTargetPositionMeters(double position) {
		Rotation2d angleSetPoint = metersToMotorRotations(position);
		motor.applyAngleRequest(positionRequest.withSetPoint(angleSetPoint));
	}

	public boolean isAtBackwardLimit() {
		return digitalInputsInputs.debouncedValue;
	}

	private Rotation2d getElevatorAngle() {
		return Rotation2d.fromRotations(elevatorStuff.motorStuff().positionSignal().getLatestValue().getRotations());
	}

	public double getPositionMeters() {
		return motorRotationsToMeters(getElevatorAngle());
	}

	protected void stayInPlace() {
		setTargetPositionMeters(getPositionMeters());
	}

	private static double motorRotationsToMeters(Rotation2d rotations) {
		return rotations.getRotations() * ElevatorConstants.MOTOR_ROTATIONS_TO_METERS_CONVERSION_RATIO;
	}

	private static Rotation2d metersToMotorRotations(double meters) {
		return Rotation2d.fromRotations(meters / ElevatorConstants.MOTOR_ROTATIONS_TO_METERS_CONVERSION_RATIO);
	}

	protected void updateInputs() {
		limitSwitch.updateInputs(digitalInputsInputs);
		motor.updateSignals(elevatorStuff.motorStuff().positionSignal(), elevatorStuff.motorStuff().voltageSignal());

		Logger.processInputs(elevatorStuff.digitalInputsLogPath(), digitalInputsInputs);
		Logger.recordOutput(getLogPath() + "isAtBackwardLimit", isAtBackwardLimit());
		Logger.recordOutput(getLogPath() + "elevatorPosition", getPositionMeters());
	}

	@Override
	protected void subsystemPeriodic() {
		if (ElevatorConstants.MINIMUM_ACHIEVABLE_POSITION_METERS > getPositionMeters()) {
			motor.resetPosition(metersToMotorRotations(ElevatorConstants.MINIMUM_ACHIEVABLE_POSITION_METERS));
		}
		updateInputs();
	}

}
