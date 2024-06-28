package frc.utils.batteryutils;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class Battery {

    public static double getDefaultBatteryVoltage() {
        return BatteryConstants.DEFAULT_BATTERY_VOLTAGE;
    }

    public static double getTotalCurrent() {
        return BatteryConstants.POWER_DISTRIBUTION.getTotalCurrent();
    }

    public static double getCurrentVoltage() {
        return RobotController.getBatteryVoltage();
    }

    public static double getMinimumVoltage() {
        return BatteryConstants.MIN_VOLTAGE_BATTERY;
    }

    public static void scheduleBatteryLimiterCommand() {
        BatteryConstants.BATTERY_LIMITER.schedule();
    }

    protected static void logBatteryStatus() {
        Logger.recordOutput(BatteryConstants.LOG_PATH + "Voltage", getCurrentVoltage());
        Logger.recordOutput(BatteryConstants.LOG_PATH + "Current", getTotalCurrent());
    }

    protected static void reportLowBatteryToLog() {
        Logger.recordOutput(BatteryConstants.ALERT_LOG_PATH + "LowVoltageAt", Timer.getFPGATimestamp());
    }

}
