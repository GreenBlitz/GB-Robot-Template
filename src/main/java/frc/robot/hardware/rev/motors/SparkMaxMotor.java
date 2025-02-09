package frc.robot.hardware.rev.motors;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.config.SparkBaseConfig;
import frc.robot.Robot;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.hardware.ConnectedInputAutoLogged;
import frc.robot.hardware.interfaces.IMotor;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.mechanisms.MechanismSimulation;
import frc.robot.hardware.rev.motors.simulation.SparkMaxSimulation;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public abstract class SparkMaxMotor implements IMotor {

	private static final int APPLY_CONFIG_RETRIES = 5;

	protected final SparkMaxWrapper motor;
	private final Optional<SparkMaxSimulation> sparkMaxSimulationOptional;
	private final String logPath;
	private final ConnectedInputAutoLogged connectedInput;
	private SparkBase.Warnings warnings;
	private SparkBase.Faults faults;

	public SparkMaxMotor(String logPath, SparkMaxWrapper motor, MechanismSimulation mechanismSimulation) {
		this.logPath = logPath;
		this.motor = motor;
		this.sparkMaxSimulationOptional = createSimulation(mechanismSimulation);
		this.warnings = motor.getWarnings();
		this.faults = motor.getFaults();

		this.connectedInput = new ConnectedInputAutoLogged();
		connectedInput.connected = true;

		createAlerts();
	}

	public SparkMaxMotor(String logPath, SparkMaxWrapper motor) {
		this(logPath, motor, null);
	}

	private Optional<SparkMaxSimulation> createSimulation(MechanismSimulation mechanismSimulation) {
		return Robot.ROBOT_TYPE.isSimulation() && mechanismSimulation != null
			? Optional.of(new SparkMaxSimulation(motor, mechanismSimulation))
			: Optional.empty();
	}

	public void createAlerts() {
		AlertManager.addAlert(new PeriodicAlert(Alert.AlertType.ERROR, logPath + "disconnectedAt", () -> !isConnected()));
		createFaultAlerts();
		createWarningAlerts();
	}

	private void createFaultAlerts() {
		//@formatter:off
		AlertManager.addAlert(
			new PeriodicAlert(
					Alert.AlertType.ERROR,
					logPath + "OtherErrorAt",
					() -> faults.other
			)
		);

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.ERROR,
				logPath + "MotorTypeMismatchAt",
				() -> faults.motorType
			)
		);

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.ERROR,
				logPath + "ConnectedSensorFaultAt",
				() -> faults.sensor
			)
		);

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.ERROR,
				logPath + "CANFatalFaultAt",
				() -> faults.can
			)
		);

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.ERROR,
				logPath + "OverHeatingAt",
				() -> faults.temperature
			)
		);

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.ERROR,
				logPath + "GateDriveCircuitryFaultAt",
				() -> faults.gateDriver
			)
		);

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.ERROR,
				logPath + "ClosedLoopControllerMemoryFaultAt",
				() -> faults.escEeprom
			)
		);

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.ERROR,
				logPath + "FirmwareFaultAt",
				() -> faults.firmware
			)
		);
		//@formatter:on
	}

	private void createWarningAlerts() {
		//@formatter:off
		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				logPath + "SignificantVoltageDropAt",
				() -> warnings.brownout
			)
		);

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				logPath + "OverCurrentDrawAt",
				() -> warnings.overcurrent
			)
		);

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				logPath + "ClosedLoopControllerMemoryWarningAt",
				() -> warnings.escEeprom
			)
		);

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				logPath + "ExternalMemoryWarningAt",
				() -> warnings.extEeprom
			)
		);

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				logPath + "ConnectedSensorWarningAt",
				() -> warnings.sensor
			)
		);

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				logPath + "MotorStalledAt",
				() -> warnings.stall
			)
		);

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				logPath + "MotorHasResetAt",
				() -> warnings.hasReset
			)
		);

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				logPath + "OtherWarningAt",
				() -> warnings.other
			)
		);
		//@formatter:on
	}

	@Override
	public void updateSimulation() {
		sparkMaxSimulationOptional.ifPresent(SparkMaxSimulation::updateMotor);
	}

	public String getLogPath() {
		return logPath;
	}

	public void applyConfiguration(SparkMaxConfiguration configuration) {
		if (motor.applyConfiguration(configuration, APPLY_CONFIG_RETRIES) != REVLibError.kOk) {
			new Alert(Alert.AlertType.ERROR, getLogPath() + "ConfigurationFailed").report();
		}
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
		if (inputSignals.length == 0) {
			return;
		}
		for (InputSignal<?> signal : inputSignals) {
			if (isValid(signal)) {
				Logger.processInputs(logPath, signal);
			} else {
				reportInvalidSignal(signal);
			}
		}

		warnings = motor.getWarnings();
		faults = motor.getFaults();
		Logger.processInputs(logPath, connectedInput);
	}

	@Override
	public void setBrake(boolean brake) {
		SparkBaseConfig.IdleMode idleMode = brake ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast;
		motor.applyConfiguration(new SparkMaxConfiguration().withSparkMaxConfig((SparkMaxConfig) new SparkMaxConfig().idleMode(idleMode)));
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
