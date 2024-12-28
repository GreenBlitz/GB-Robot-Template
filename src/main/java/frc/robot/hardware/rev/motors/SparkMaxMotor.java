package frc.robot.hardware.rev.motors;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import frc.robot.hardware.ConnectedInputAutoLogged;
import frc.robot.hardware.interfaces.IMotor;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;
import org.littletonrobotics.junction.Logger;

public abstract class SparkMaxMotor implements IMotor {

	protected final SparkMaxWrapper motor;
	private final String logPath;
	private final ConnectedInputAutoLogged connectedInput;
	private SparkBase.Warnings motorWarnings;
	private SparkBase.Faults motorFaults;

	public SparkMaxMotor(String logPath, SparkMaxWrapper motor) {
		this.logPath = logPath;
		this.motor = motor;
		this.motorWarnings = motor.getWarnings();
		this.motorFaults = motor.getFaults();

		this.connectedInput = new ConnectedInputAutoLogged();
		connectedInput.connected = true;
	}

	public void createFaultAlerts() {
		//@formatter:off
		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.ERROR,
						logPath + "disconnectedAt",
						() -> !isConnected()
				)
		);

		// The controller had detected a fault condition that doesn’t fall into any of the specific predefined fault categories.
		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.ERROR,
						logPath + "OtherErrorAt",
						() -> motorFaults.other
				)
		);

		// The controller has detected there is a mismatch between the expected and actual motor type configuration in the controller.
		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.ERROR,
						logPath + "MotorTypeAt",
						() -> motorFaults.motorType
				)
		);

		/*
		 The controller has detected an issue with one of the connected sensors,
		 such as an encoder, Hall effect sensor, or other feedback devices.
		 */
		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.ERROR,
						logPath + "SensorAt",
						() -> motorFaults.sensor
				)
		);

		// The controller has detected a fatal fault with the CANBus
		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.ERROR,
						logPath + "CANAt",
						() -> motorFaults.can
				)
		);

		// Indicates the internal temperature of the motor controller exceeds its safe operating limits.
		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.ERROR,
						logPath + "TemperatureAt",
						() -> motorFaults.temperature
				)
		);

		// Indicated there is an issue with the gate driver circuitry in the controller.
		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.ERROR,
						logPath + "GateDriveAt",
						() -> motorFaults.gateDriver
				)
		);

		// A fatal fault logged by the Electronic Speed Controller Electrically Erasable Programmable Read-Only Memory.
		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.ERROR,
						logPath + "EscEepromAt",
						() -> motorFaults.escEeprom
				)
		);

		// Indicates a fault or a malfunction related to the motor controller’s firmware
		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.ERROR,
						logPath + "FirmwareAt",
						() -> motorFaults.firmware
				)
		);
		//@formatter:on
	}

	public void createWarningAlerts() {
		//@formatter:off

		// Indicates a significant drop in the input voltage.
		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "SignificantDropInInputVoltage",
						() -> motorWarnings.brownout
				)
		);

		// Current draw is nearing or exceeding the controller’s configured or safe operating limits.
		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "OverCurrentAt",
						() -> motorWarnings.overcurrent
				)
		);

		// A warning logged by the Electronic Speed Controller Electrically Erasable Programmable Read-Only Memory.
		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "EscEepromAt",
						() -> motorWarnings.escEeprom
				)
		);

		// A warning logged by the External Electrically Erasable Programmable Read-Only Memory.
		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "ExtEepromAt",
						() -> motorWarnings.extEeprom
				)
		);

		// The controller has detected an issue with a connected sensor, such as an encoder or other feedback device.
		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "SensorAt",
						() -> motorWarnings.sensor
				)
		);

		// The controller has detected conditions consistent with a motor being stalled.
		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "StallAt",
						() -> motorWarnings.stall
				)
		);

		/*
		 The controller has experienced a reset event since the last time it was checked.
		 This reset could be the result of several factors, such as power loss, firmware issues,
		 or intentional user actions like rebooting the controller.
		 */
		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "HasResetAt",
						() -> motorWarnings.hasReset
				)
		);

		// The controller has detected a generic warning that doesn’t fall into specific predefined categories.
		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "OtherWarningAt",
						() -> motorWarnings.other
				)
		);
		//@formatter:on
	}

	@Override
	public void updateSimulation() {}

	public String getLogPath() {
		return logPath;
	}

	@Override
	public boolean isConnected() {
		return connectedInput.connected;
	}

	private boolean isValid(InputSignal<?> signal) {
		return signal instanceof SuppliedDoubleSignal || signal instanceof SuppliedAngleSignal;
	}

	private void reportInvalidSignal(InputSignal<?> invalidSignal) {
		new Alert(
			Alert.AlertType.WARNING,
			logPath + "signal named " + invalidSignal.getName() + " has invalid type " + invalidSignal.getClass().getSimpleName()
		).report();
	}

	@Override
	public void updateInputs(InputSignal<?>... inputSignals) {
		for (InputSignal<?> signal : inputSignals) {
			if (isValid(signal)) {
				Logger.processInputs(logPath, signal);
			} else {
				reportInvalidSignal(signal);
			}
		}

		motorWarnings = motor.getWarnings();
		motorFaults = motor.getFaults();
		Logger.processInputs(logPath, connectedInput);
	}

	@Override
	public void setBrake(boolean brake) {
		SparkBaseConfig.IdleMode idleMode = brake ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast;

		// TODO motor.configure()
//		motor.setIdleMode(idleMode);
	}

	@Override
	public void stop() {
		motor.stopMotor();
	}

	@Override
	public void setPower(double power) {
		motor.set(power);
	}

}
