package frc.utils.battery;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
