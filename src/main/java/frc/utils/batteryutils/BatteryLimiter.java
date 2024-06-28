package frc.utils.batteryutils;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants;
import frc.utils.DriverStationUtils;

class BatteryLimiter extends Command {

    private final LinearFilter voltageFilter;

    public BatteryLimiter() {
        this.voltageFilter = LinearFilter.movingAverage(BatteryConstants.NUMBER_OF_VALUES_TAKEN_IN_AVERAGE);
    }

    @Override
    public void initialize() {
        // Fill linear filter with battery voltage values instead of 1/NUMBER_OF_VALUES_IN_AVERAGE
        for (int i = 0; i < BatteryConstants.NUMBER_OF_VALUES_TAKEN_IN_AVERAGE; i++) {
            voltageFilter.calculate(Battery.getCurrentVoltage());
        }
    }

    @Override
    public void execute() {
        Battery.logBatteryStatus();

        double currentAverageVoltage = voltageFilter.calculate(Battery.getCurrentVoltage());
        if (currentAverageVoltage <= Battery.getMinimumVoltage()) {
            Battery.reportLowBatteryToLog();
            if (!DriverStationUtils.isMatch() && RobotConstants.ENABLE_BATTERY_LIMITER) {
                throw new java.lang.RuntimeException("BATTERY IS LOW");
            }
        }
    }

}
