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
	private final ControllableMotor firstMotor;
	private final ControllableMotor secondMotor;
	private final IDigitalInput limitSwitch;

	public Elevator(ElevatorStuff elevatorStuff) {
		super(elevatorStuff.logPath());

		this.firstMotor = elevatorStuff.firstMotorStuff().motor();
		this.secondMotor = elevatorStuff.secondMotorStuff().motor();
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
		secondMotor.setPower(power);
	}

	public void stop() {
		firstMotor.stop();
		secondMotor.stop();
	}

	public void setBrake(boolean brake) {
		firstMotor.setBrake(brake);
		secondMotor.setBrake(brake);
	}

	public void setTargetAngle(Rotation2d angle) {
		firstMotor.applyAngleRequest(angleRequest.withSetPoint(angle));
		secondMotor.applyAngleRequest(angleRequest.withSetPoint(angle));
	}

	public boolean isAtBackwardLimit() {
		return digitalInputsInputs.debouncedValue;
	}

	public Rotation2d getElevatorPosition() {
		return Rotation2d.fromRotations((elevatorStuff.firstMotorStuff().motorPositionSignal().getLatestValue().getRotations() + elevatorStuff.secondMotorStuff().motorPositionSignal().getLatestValue().getRotations()) / 2)
	}

	public void stayInPlace() {
		firstMotor.applyAngleRequest(angleRequest.withSetPoint(elevatorStuff.firstMotorStuff().motorPositionSignal().getLatestValue()));
		secondMotor.applyAngleRequest(angleRequest.withSetPoint(elevatorStuff.secondMotorStuff().motorPositionSignal().getLatestValue()));
	}

	public double rotationsToMeters(Rotation2d rotations) {
		return rotations.getRotations() * ElevatorConstants.GEAR_RATIO * elevatorStuff.rotationsToMetersConversionRatio();
	}

	public void updateInputs() {
		limitSwitch.updateInputs(digitalInputsInputs);
		firstMotor.updateSignals(elevatorStuff.firstMotorStuff().motorPositionSignal(), elevatorStuff.firstMotorStuff().voltageSignal());
		secondMotor.updateSignals(elevatorStuff.secondMotorStuff().motorPositionSignal(), elevatorStuff.secondMotorStuff().voltageSignal());
	}

	public void logState() {
		Logger.processInputs(elevatorStuff.digitalInputsLogPath(), digitalInputsInputs);
		Logger.recordOutput(this.getLogPath() + "isAtBackwardLimit", isAtBackwardLimit());
		Logger.recordOutput(this.getLogPath() + "elevatorPosition", rotationsToMeters(getElevatorPosition()));
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
		logState();
	}

}
