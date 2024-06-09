package frc.utils.batteryutils;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants;
import frc.utils.DriverStationUtils;
import frc.utils.dashboard.LoggedTableBoolean;

class BatteryLimiter extends Command {

    private final LoggedTableBoolean isBatteryLow = new LoggedTableBoolean("Battery", "is low", false);

    private final LinearFilter voltageFilter;

    private boolean showedMessage; // todo - maybe add that image is showed every 5 minutes (like reminder)

    public BatteryLimiter() {
        this.voltageFilter = LinearFilter.movingAverage(BatteryConstants.NUMBER_OF_VALUES_IN_AVERAGE);
        this.showedMessage = false;
//        CMDHandler.runCMDCommand(BatteryConstants.SHOW_BATTERY_MESSAGE_COMMAND);
    }

    private void showBatteryMessage() {
        isBatteryLow.set(true);
        showedMessage = true;
    }

    @Override
    public void initialize() {
        // Fill linear filter with battery voltage values instead of 1/NUMBER_OF_VALUES_IN_AVERAGE
        for (int i = 0; i < BatteryConstants.NUMBER_OF_VALUES_IN_AVERAGE; i++) {
            voltageFilter.calculate(Battery.getCurrentVoltage());
        }
    }

    @Override
    public void execute() {
        Battery.logBatteryStatus();

        double currentAverageVoltage = voltageFilter.calculate(Battery.getCurrentVoltage());
        if (currentAverageVoltage <= Battery.getMinimumVoltage()) {
            Battery.reportAlertsToLog();
            if (!DriverStationUtils.isGame()) {
                showBatteryMessage();
                if (RobotConstants.ENABLE_BATTERY_LIMITER) {
                    throw new java.lang.RuntimeException("BATTERY IS LOW");
                }
            }
        }
    }

}
