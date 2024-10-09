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
	private final ElevatorCommandsBuilder commandBuilder;
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
		this.commandBuilder = new ElevatorCommandsBuilder(this);
		this.positionRequest = elevatorStuff.positionRequest();
		this.voltageRequest = elevatorStuff.voltageRequest();
	}

	public ElevatorCommandsBuilder getCommandBuilder() {
		return commandBuilder;
	}

	public void setPower(double power) {
		frontMotor.setPower(power);
		backMotor.setPower(power);
	}

	public void setVoltage(double voltage) {
		frontMotor.applyDoubleRequest(voltageRequest.withSetPoint(voltage));
		backMotor.applyDoubleRequest(voltageRequest.withSetPoint(voltage));
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
		return motorRotationsToMeters(
			Rotation2d.fromRotations(
				(elevatorStuff.frontMotorStuff().positionSignal().getLatestValue().getRotations()
					+ elevatorStuff.backMotorStuff().positionSignal().getLatestValue().getRotations()) / 2
			)
		);
	}

	public void stayInPlace() {
		frontMotor.applyAngleRequest(positionRequest.withSetPoint(elevatorStuff.frontMotorStuff().positionSignal().getLatestValue()));
		backMotor.applyAngleRequest(positionRequest.withSetPoint(elevatorStuff.backMotorStuff().positionSignal().getLatestValue()));
	}

	public double motorRotationsToMeters(Rotation2d rotations) {
		return rotations.getRotations() * elevatorStuff.motorRotationsToMetersConversionRatio();
	}

	public void updateInputs() {
		limitSwitch.updateInputs(digitalInputsInputs);
		frontMotor.updateSignals(elevatorStuff.frontMotorStuff().positionSignal(), elevatorStuff.frontMotorStuff().voltageSignal());
		backMotor.updateSignals(elevatorStuff.backMotorStuff().positionSignal(), elevatorStuff.backMotorStuff().voltageSignal());

		Logger.processInputs(elevatorStuff.digitalInputsLogPath(), digitalInputsInputs);
		Logger.recordOutput(this.getLogPath() + "isAtBackwardLimit", isAtBackwardLimit());
		Logger.recordOutput(this.getLogPath() + "elevatorPosition", getPositionMeters());
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
	}

}
