package frc.utils.batteryutils;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class Battery extends GBSubsystem {

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

    public static Command getBatteryLimiterCommand() {
        return BatteryConstants.BATTERY_LIMITER;
    }

    public static void scheduleBatteryLimiterCommand() {
        BatteryConstants.BATTERY_LIMITER.schedule();
    }

    public static void logBatteryStatus() {
        Logger.recordOutput(BatteryConstants.LOG_PATH + "Voltage", getCurrentVoltage());
        Logger.recordOutput(BatteryConstants.LOG_PATH + "Current", getTotalCurrent());
    }

}
