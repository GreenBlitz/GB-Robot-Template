package frc.robot.subsystems.motorSubsystem;

import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;

public class MotorSubsystem extends GBSubsystem {

	private final ControllableMotor motor;
	private final MotorCommandBuilder commandBuilder;
	private IRequest<Double> voltageRequest;
	private InputSignal<Double> voltageSignal;


	public MotorSubsystem(ControllableMotor motor, String logPath) {
		super(logPath);
		this.motor = motor;
		this.commandBuilder = new MotorCommandBuilder(this);
	}

	public MotorSubsystem withVoltageControl(IRequest<Double> voltageRequest, InputSignal<Double> voltageSignal) {
		this.voltageRequest = voltageRequest;
		this.voltageSignal = voltageSignal;
		return this;
	}

	public Double getAppliedVoltage() {
		if (voltageSignal == null) {
			throw new NullPointerException("The voltage signal is null, try using: '.withVoltageControl'");
		}
		return voltageSignal.getLatestValue();
	}

	public void stop() {
		motor.stop();
	}

	public MotorCommandBuilder getCommandBuilder() {
		return commandBuilder;
	}

	public void setVoltage(double voltage) {
		if (voltageRequest == null) {
			throw new NullPointerException("The voltage request is null, try using: '.withVoltageControl'");
		}
		this.voltageRequest.withSetPoint(voltage);
	}

	public void setPower(Double power) {
		motor.setPower(power);
	}


	public void updateInputs() {
		motor.updateInputs(voltageSignal);
	}

	public void applyRequests() {
		motor.applyRequest(voltageRequest);
	}


}
