package frc.robot.hardware.rev.motors;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.util.Signal;
import frc.robot.Robot;
import com.revrobotics.spark.config.SparkMaxConfig;
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
	private Signal<SparkBase.Warnings> warnings;
	private Signal<SparkBase.Faults> faults;

	public SparkMaxMotor(String logPath, SparkMaxWrapper motor, MechanismSimulation mechanismSimulation) {
		this.logPath = logPath;
		this.motor = motor;
		this.sparkMaxSimulationOptional = createSimulation(mechanismSimulation);
		this.warnings = motor.getWarnings();
		this.faults = motor.getFaults();

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
		createFaultAlerts();
		createWarningAlerts();
	}

	private void createFaultAlerts() {
		//@formatter:off
		AlertManager.addAlert(
			new PeriodicAlert(
					Alert.AlertType.ERROR,
					logPath + "OtherErrorAt",
					() -> faults.get().other
			)
		);

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.ERROR,
				logPath + "MotorTypeMismatchAt",
				() -> faults.get().motorType
			)
		);

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.ERROR,
				logPath + "ConnectedSensorFaultAt",
				() -> faults.get().sensor
			)
		);

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.ERROR,
				logPath + "CANFatalFaultAt",
				() -> faults.get().can
			)
		);

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.ERROR,
				logPath + "OverHeatingAt",
				() -> faults.get().temperature
			)
		);

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.ERROR,
				logPath + "GateDriveCircuitryFaultAt",
				() -> faults.get().gateDriver
			)
		);

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.ERROR,
				logPath + "ClosedLoopControllerMemoryFaultAt",
				() -> faults.get().escEeprom
			)
		);

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.ERROR,
				logPath + "FirmwareFaultAt",
				() -> faults.get().firmware
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
				() -> warnings.get().brownout
			)
		);

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				logPath + "OverCurrentDrawAt",
				() -> warnings.get().overcurrent
			)
		);

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				logPath + "ClosedLoopControllerMemoryWarningAt",
				() -> warnings.get().escEeprom
			)
		);

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				logPath + "ExternalMemoryWarningAt",
				() -> warnings.get().extEeprom
			)
		);

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				logPath + "ConnectedSensorWarningAt",
				() -> warnings.get().sensor
			)
		);

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				logPath + "MotorStalledAt",
				() -> warnings.get().stall
			)
		);

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				logPath + "MotorHasResetAt",
				() -> warnings.get().hasReset
			)
		);

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				logPath + "OtherWarningAt",
				() -> warnings.get().other
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
		return true;
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
		warnings = motor.getWarnings();
		faults = motor.getFaults();

		for (InputSignal<?> signal : inputSignals) {
			if (isValid(signal)) {
				Logger.processInputs(logPath, signal);
			} else {
				reportInvalidSignal(signal);
			}
		}
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
		motor.setThrottle(power);
	}

}
