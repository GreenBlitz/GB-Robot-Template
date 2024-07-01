package frc.utils.batteryutils;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class Battery {

    public static double getDefaultVoltage() {
        return BatteryConstants.DEFAULT_VOLTAGE;
    }

    public static double getTotalCurrent() {
        return BatteryConstants.POWER_DISTRIBUTION.getTotalCurrent();
    }

    public static double getCurrentVoltage() {
        return RobotController.getBatteryVoltage();
    }

    public static double getMinimumVoltage() {
        return BatteryConstants.MIN_VOLTAGE;
    }

    public static void scheduleLimiter() {
        BatteryConstants.LIMITER.schedule();
    }

    protected static void logStatus() {
        Logger.recordOutput(BatteryConstants.LOG_PATH + "Voltage", getCurrentVoltage());
        Logger.recordOutput(BatteryConstants.LOG_PATH + "Current", getTotalCurrent());
    }

    protected static void reportLowBattery() {
        Logger.recordOutput(BatteryConstants.ALERT_LOG_PATH + "LowVoltageAt", Timer.getFPGATimestamp());
    }

}
