package frc.robot.subsystems.motorSubsystem;

import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.alerts.Alert;

public class MotorSubsystem extends GBSubsystem {

	private final ControllableMotor motor;
	private final MotorCommandsBuilder commandsBuilder;
	private IRequest<Double> voltageRequest;
	private InputSignal<Double> voltageSignal;
	private IRequest<?> lastChangedRequest;
	private boolean allRequestsAreApplied;


	public MotorSubsystem(ControllableMotor motor, String logPath) {
		super(logPath);
		this.motor = motor;
		this.commandsBuilder = new MotorCommandsBuilder(this);
		this.allRequestsAreApplied = true;
	}

	public MotorCommandsBuilder getCommandBuilder() {
		return commandsBuilder;
	}

	public MotorSubsystem withVoltageControl(IRequest<Double> voltageRequest, InputSignal<Double> voltageSignal) {
		this.voltageRequest = voltageRequest;
		this.voltageSignal = voltageSignal;
		return this;
	}

	public Double getVoltage() {
		if (voltageSignal == null) {
			new Alert(Alert.AlertType.ERROR, "The voltage signal is null. try using '.withVoltageControl'").report();
			return null;
		}
		return voltageSignal.getLatestValue();
	}

	public void setVoltage(double voltage) {
		if (voltageRequest == null) {
			new Alert(Alert.AlertType.ERROR, "The voltage request is null. try using '.withVoltageControl'").report();
		} else {
			this.voltageRequest.withSetPoint(voltage);
			lastChangedRequest = voltageRequest;
		}
	}

	public void stop() {
		motor.stop();
	}

	public void setPower(double power) {
		motor.setPower(power);
	}

	public void updateInputs() {
		motor.updateInputs(voltageSignal);
	}

	private void reapplyRequest() {
		if (!motor.isConnected()) {
			allRequestsAreApplied = false;
		}
		if (!allRequestsAreApplied && motor.isConnected()) {
			motor.applyRequest(lastChangedRequest);
			allRequestsAreApplied = true;
		}
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
		reapplyRequest();
	}

}
