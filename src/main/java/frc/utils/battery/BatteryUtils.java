package frc.utils.battery;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

import static frc.utils.battery.BatteryConstants.*;

public class BatteryUtils {

	public static final double DEFAULT_VOLTAGE = 12;
	public static final double MIN_VOLTAGE = 10.5;

	private static final PowerDistribution powerDistribution = new PowerDistribution(
		POWER_DISTRIBUTION_DEVICE_ID.getId(),
		POWER_DISTRIBUTION_DEVICE_ID.getType()
	);
	private static final Command limiter = new BatteryLimiter().ignoringDisable(true);


	public static double getTotalCurrent() {
		return powerDistribution.getTotalCurrent();
	}

	public static double getCurrentVoltage() {
		return RobotController.getBatteryVoltage();
	}

	public static void scheduleLimiter() {
		if (!limiter.isScheduled()) {
			limiter.schedule();
		}
	}

	public static void logStatus() {
		Logger.recordOutput(BatteryConstants.LOG_PATH + "Voltage", getCurrentVoltage());
		Logger.recordOutput(BatteryConstants.LOG_PATH + "Current", getTotalCurrent());
	}

}
