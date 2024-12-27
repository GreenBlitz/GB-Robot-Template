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

	public SparkMaxMotor(String logPath, SparkMaxWrapper motor) {
		this.logPath = logPath;
		this.motor = motor;

		this.connectedInput = new ConnectedInputAutoLogged();
		connectedInput.connected = true;
	}

	public void createAlerts() {
		SparkBase.Warnings warnings = motor.getWarnings();

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
						Alert.AlertType.WARNING,
						logPath + "BrownoutAt",
						() -> warnings.brownout
				)
		);

		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "OverCurrentAt",
						() -> warnings.overcurrent
				)
		);

		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "EscEepromAt",
						() -> warnings.escEeprom
				)
		);

		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "ExtEepromAt",
						() -> warnings.extEeprom
				)
		);

		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "SensorAt",
						() -> warnings.sensor
				)
		);

		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "StallAt",
						() -> warnings.stall
				)
		);

		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "HasResetAt",
						() -> warnings.hasReset
				)
		);

		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "OtherAt",
						() -> warnings.other
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
