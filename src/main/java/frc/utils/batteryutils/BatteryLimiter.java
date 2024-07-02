package frc.utils.batteryutils;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants;
import frc.utils.DriverStationUtils;
import org.littletonrobotics.junction.Logger;

class BatteryLimiter extends Command {

    private static final int NUMBER_OF_SAMPLES_TAKEN_IN_AVERAGE = 50;

    private final LinearFilter voltageFilter;

    public BatteryLimiter() {
        this.voltageFilter = LinearFilter.movingAverage(NUMBER_OF_SAMPLES_TAKEN_IN_AVERAGE);
    }

    private static void reportLowBattery() {
        Logger.recordOutput(BatteryConstants.ALERT_LOG_PATH + "LowVoltageAt", Timer.getFPGATimestamp());
    }

    @Override
    public void initialize() {
        // Fill linear filter with battery voltage values instead of 1/NUMBER_OF_VALUES_IN_AVERAGE
        for (int i = 0; i < NUMBER_OF_SAMPLES_TAKEN_IN_AVERAGE; i++) {
            voltageFilter.calculate(BatteryUtils.getCurrentVoltage());
        }
    }

    @Override
    public void execute() {
        double currentAverageVoltage = voltageFilter.calculate(BatteryUtils.getCurrentVoltage());
        if (currentAverageVoltage <= BatteryUtils.MIN_VOLTAGE) {
            reportLowBattery();
            if (RobotConstants.ENABLE_BATTERY_LIMITER && !DriverStationUtils.isMatch()) {
                throw new java.lang.RuntimeException("BATTERY IS LOW");
            }
        }
    }

}
