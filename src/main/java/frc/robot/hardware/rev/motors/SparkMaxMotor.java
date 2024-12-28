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

		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.ERROR,
						logPath + "OtherErrorAt",
						() -> motorFaults.other
				)
		);

		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.ERROR,
						logPath + "MotorTypeAt",
						() -> motorFaults.motorType
				)
		);

		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.ERROR,
						logPath + "SensorAt",
						() -> motorFaults.sensor
				)
		);

		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.ERROR,
						logPath + "CANAt",
						() -> motorFaults.can
				)
		);

		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.ERROR,
						logPath + "TemperatureAt",
						() -> motorFaults.temperature
				)
		);

		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.ERROR,
						logPath + "GateDriveAt",
						() -> motorFaults.gateDriver
				)
		);

		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.ERROR,
						logPath + "EscEepromAt",
						() -> motorFaults.escEeprom
				)
		);

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

		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "BrownoutAt",
						() -> motorWarnings.brownout
				)
		);

		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "OverCurrentAt",
						() -> motorWarnings.overcurrent
				)
		);

		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "EscEepromAt",
						() -> motorWarnings.escEeprom
				)
		);

		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "ExtEepromAt",
						() -> motorWarnings.extEeprom
				)
		);

		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "SensorAt",
						() -> motorWarnings.sensor
				)
		);

		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "StallAt",
						() -> motorWarnings.stall
				)
		);

		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "HasResetAt",
						() -> motorWarnings.hasReset
				)
		);

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
