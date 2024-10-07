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
	private final ElevatorCommandBuilder commandBuilder;
	private final IRequest<Rotation2d> angleRequest;

	private final ElevatorStuff elevatorStuff;
	private final ControllableMotor frontMotor;
	private final ControllableMotor backwardMotor;
	private final IDigitalInput limitSwitch;

	public Elevator(ElevatorStuff elevatorStuff) {
		super(elevatorStuff.logPath());

		this.frontMotor = elevatorStuff.frontMotorStuff().motor();
		this.backwardMotor = elevatorStuff.backwardMotorStuff().motor();
		this.limitSwitch = elevatorStuff.digitalInput();

		this.digitalInputsInputs = new DigitalInputInputsAutoLogged();
		this.elevatorStuff = elevatorStuff;
		this.commandBuilder = new ElevatorCommandBuilder(this);
		this.angleRequest = elevatorStuff.angleRequest();
	}

	public ElevatorCommandBuilder getCommandBuilder() {
		return commandBuilder;
	}

	public void setPower(double power) {
		frontMotor.setPower(power);
		backwardMotor.setPower(power);
	}

	public void stop() {
		frontMotor.stop();
		backwardMotor.stop();
	}

	public void setBrake(boolean brake) {
		frontMotor.setBrake(brake);
		backwardMotor.setBrake(brake);
	}

	public void setTargetAngle(Rotation2d angle) {
		frontMotor.applyAngleRequest(angleRequest.withSetPoint(angle));
		backwardMotor.applyAngleRequest(angleRequest.withSetPoint(angle));
	}

	public boolean isAtBackwardLimit() {
		return digitalInputsInputs.debouncedValue;
	}

	public double getElevatorPositionMeters() {
		return rotationsToMeters(
			Rotation2d.fromRotations(
				(elevatorStuff.frontMotorStuff().motorPositionSignal().getLatestValue().getRotations()
					+ elevatorStuff.backwardMotorStuff().motorPositionSignal().getLatestValue().getRotations()) / 2
			)
		);
	}

	public void stayInPlace() {
		frontMotor.applyAngleRequest(angleRequest.withSetPoint(elevatorStuff.frontMotorStuff().motorPositionSignal().getLatestValue()));
		backwardMotor.applyAngleRequest(angleRequest.withSetPoint(elevatorStuff.backwardMotorStuff().motorPositionSignal().getLatestValue()));
	}

	public double rotationsToMeters(Rotation2d rotations) {
		return rotations.getRotations() * ElevatorConstants.GEAR_RATIO * elevatorStuff.rotationsToMetersConversionRatio();
	}

	public void updateInputs() {
		limitSwitch.updateInputs(digitalInputsInputs);
		frontMotor.updateSignals(elevatorStuff.frontMotorStuff().motorPositionSignal(), elevatorStuff.frontMotorStuff().voltageSignal());
		backwardMotor
			.updateSignals(elevatorStuff.backwardMotorStuff().motorPositionSignal(), elevatorStuff.backwardMotorStuff().voltageSignal());
	}

	public void logState() {
		Logger.processInputs(elevatorStuff.digitalInputsLogPath(), digitalInputsInputs);
		Logger.recordOutput(this.getLogPath() + "isAtBackwardLimit", isAtBackwardLimit());
		Logger.recordOutput(this.getLogPath() + "elevatorPosition", getElevatorPositionMeters());
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
		logState();
	}

}
