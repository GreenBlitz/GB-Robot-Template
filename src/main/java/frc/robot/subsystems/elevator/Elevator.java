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
	private final IRequest<Rotation2d> positionRequest;
	private final IRequest<Double> voltageRequest;

	private final ElevatorStuff elevatorStuff;
	private final ControllableMotor frontMotor;
	private final ControllableMotor backMotor;
	private final IDigitalInput limitSwitch;

	public Elevator(ElevatorStuff elevatorStuff) {
		super(elevatorStuff.logPath());

		this.frontMotor = elevatorStuff.frontMotorStuff().motor();
		this.backMotor = elevatorStuff.backMotorStuff().motor();
		this.limitSwitch = elevatorStuff.digitalInput();

		this.digitalInputsInputs = new DigitalInputInputsAutoLogged();
		this.elevatorStuff = elevatorStuff;
		this.commandBuilder = new ElevatorCommandBuilder(this);
		this.positionRequest = elevatorStuff.angleRequest();
		this.voltageRequest = elevatorStuff.voltageRequest();
	}

	public ElevatorCommandBuilder getCommandBuilder() {
		return commandBuilder;
	}

	public void setPower(double power) {
		frontMotor.setPower(power);
		backMotor.setPower(power);
	}

	public void setVoltage(double voltage) {
		frontMotor.applyDoubleRequest(voltageRequest);
		backMotor.applyDoubleRequest(voltageRequest);
	}

	public void stop() {
		frontMotor.stop();
		backMotor.stop();
	}

	public void setBrake(boolean brake) {
		frontMotor.setBrake(brake);
		backMotor.setBrake(brake);
	}

	public void setTargetPosition(Rotation2d position) {
		frontMotor.applyAngleRequest(positionRequest.withSetPoint(position));
		backMotor.applyAngleRequest(positionRequest.withSetPoint(position));
	}

	public boolean isAtBackwardLimit() {
		return digitalInputsInputs.debouncedValue;
	}

	public double getPositionMeters() {
		return rotationsToMeters(
			Rotation2d.fromRotations(
				(elevatorStuff.frontMotorStuff().motorPositionSignal().getLatestValue().getRotations()
					+ elevatorStuff.backMotorStuff().motorPositionSignal().getLatestValue().getRotations()) / 2
			)
		);
	}

	public void stayInPlace() {
		frontMotor.applyAngleRequest(positionRequest.withSetPoint(elevatorStuff.frontMotorStuff().motorPositionSignal().getLatestValue()));
		backMotor.applyAngleRequest(positionRequest.withSetPoint(elevatorStuff.backMotorStuff().motorPositionSignal().getLatestValue()));
	}

	public double rotationsToMeters(Rotation2d rotations) {
		return rotations.getRotations() * elevatorStuff.motorRotationsToMetersConversionRatio();
	}

	public void updateInputs() {
		limitSwitch.updateInputs(digitalInputsInputs);
		frontMotor.updateSignals(elevatorStuff.frontMotorStuff().motorPositionSignal(), elevatorStuff.frontMotorStuff().voltageSignal());
		backMotor.updateSignals(elevatorStuff.backMotorStuff().motorPositionSignal(), elevatorStuff.backMotorStuff().voltageSignal());

		Logger.processInputs(elevatorStuff.digitalInputsLogPath(), digitalInputsInputs);
		Logger.recordOutput(this.getLogPath() + "isAtBackwardLimit", isAtBackwardLimit());
		Logger.recordOutput(this.getLogPath() + "elevatorPosition", getPositionMeters());
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
	}

}
