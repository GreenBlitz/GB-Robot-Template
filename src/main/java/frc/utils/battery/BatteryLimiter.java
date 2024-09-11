package frc.utils.battery;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.IPs;
import frc.utils.CMDHandler;
import frc.utils.DriverStationUtils;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;

import java.nio.file.Path;

class BatteryLimiter extends Command {

	private static final int NUMBER_OF_SAMPLES_TAKEN_IN_AVERAGE = 50;
	private static final String LOW_BATTERY_TOPIC_NAME = "BatteryMessage/LowBattery";

	private final BooleanEntry lowBatteryEntry;
	private final LinearFilter voltageFilter;
	private double averageVoltage;

	public BatteryLimiter() {
		this.voltageFilter = LinearFilter.movingAverage(NUMBER_OF_SAMPLES_TAKEN_IN_AVERAGE);
		this.averageVoltage = BatteryUtils.getCurrentVoltage();
		this.lowBatteryEntry = NetworkTableInstance.getDefault().getBooleanTopic(LOW_BATTERY_TOPIC_NAME).getEntry(false);
		lowBatteryEntry.set(false);

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				BatteryConstants.LOG_PATH + "LowVoltageAt",
				() -> averageVoltage <= BatteryUtils.MIN_VOLTAGE
			)
		);

		if (Robot.ROBOT_TYPE.isSimulation()) {
			CMDHandler.runPythonScript(Path.of("BatteryMessage"), IPs.SIMULATION_IP);
		}
	}

	private void setVoltageFilterToCurrentVoltage() {
		// Fills linear filter with battery voltage values instead of 1/NUMBER_OF_VALUES_IN_AVERAGE
		for (int i = 0; i < NUMBER_OF_SAMPLES_TAKEN_IN_AVERAGE; i++) {
			voltageFilter.calculate(BatteryUtils.getCurrentVoltage());
		}
	}

	private void showBatteryMessage() {
		if (!lowBatteryEntry.get()) {
			lowBatteryEntry.set(true);
		}
	}

	@Override
	public void initialize() {
		setVoltageFilterToCurrentVoltage();
	}

	@Override
	public void execute() {
		averageVoltage = voltageFilter.calculate(BatteryUtils.getCurrentVoltage());
		if (averageVoltage <= BatteryUtils.MIN_VOLTAGE && !DriverStationUtils.isMatch()) {
			showBatteryMessage();
		} else if (lowBatteryEntry.get()) {
			lowBatteryEntry.set(false);
		}
	}

}
