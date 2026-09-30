package frc.utils.battery;

import org.wpilib.system.RobotController;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.littletonrobotics.junction.Logger;

public class BatteryUtil {

	public static final double DEFAULT_VOLTAGE = 12;
	public static final double MIN_VOLTAGE = 10.5;

	private static final Command limiter = new BatteryLimiter().ignoringDisable(true);

	public static double getCurrentVoltage() {
		return RobotController.getBatteryVoltage();
	}

	public static void scheduleLimiter() {
		if (!limiter.isScheduled()) {
			CommandScheduler.getInstance().schedule(limiter);
		}
	}

	public static void logStatus() {
		Logger.recordOutput(BatteryConstants.LOG_PATH + "/Voltage", getCurrentVoltage());
	}

}
