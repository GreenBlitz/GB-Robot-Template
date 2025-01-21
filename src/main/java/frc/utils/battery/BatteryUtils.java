package frc.utils.battery;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.IDs;
import org.littletonrobotics.junction.Logger;

public class BatteryUtils {

	public static final double DEFAULT_VOLTAGE = 12;
	public static final double MIN_VOLTAGE = 10.5;

	private static final PowerDistribution powerDistribution = new PowerDistribution(
		IDs.POWER_DISTRIBUTION_DEVICE_ID.ID(),
		IDs.POWER_DISTRIBUTION_DEVICE_ID.type()
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
		Logger.recordOutput(BatteryConstants.LOG_PATH + "/Voltage", getCurrentVoltage());
		Logger.recordOutput(BatteryConstants.LOG_PATH + "/Current", getTotalCurrent());
	}

}
