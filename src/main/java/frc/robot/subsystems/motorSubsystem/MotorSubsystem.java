package frc.robot.subsystems.motorSubsystem;

import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;

public class MotorSubsystem extends GBSubsystem {

	private final ControllableMotor motor;
	private final MotorCommandBuilder commandBuilder;
	private IRequest<Double> voltageRequest;
	private InputSignal<Double> voltageSignal;
	private IRequest lastChangedRequest;
	private boolean wasRequestApllied;


	public MotorSubsystem(ControllableMotor motor, String logPath) {
		super(logPath);
		this.motor = motor;
		this.commandBuilder = new MotorCommandBuilder(this);
		this.wasRequestApllied = true;
	}

	public MotorSubsystem withVoltageControl(IRequest<Double> voltageRequest, InputSignal<Double> voltageSignal) {
		this.voltageRequest = voltageRequest;
		this.voltageSignal = voltageSignal;
		return this;
	}

	public double getAppliedVoltage() {
		if (voltageSignal == null) {
			new Alert(Alert.AlertType.ERROR,"The voltage signal is null. try using '.withVoltageControl'").report();
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
			new Alert(Alert.AlertType.ERROR,"The voltage signal is null. try using '.withVoltageControl'").report();
		}
		this.voltageRequest.withSetPoint(voltage);
		lastChangedRequest = voltageRequest;
	}

	public void setPower(Double power) {
		motor.setPower(power);
	}

	public void updateInputs() {
		motor.updateInputs(voltageSignal);
	}

	private void reapplyRequest() {
		if (!motor.isConnected()) {
			wasRequestApllied = false;
		}
		if (!wasRequestApllied&& motor.isConnected()) {
			motor.applyRequest(lastChangedRequest);
			wasRequestApllied = true;
		}
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
		reapplyRequest();
	}

}
