package frc.utils.batteryutils;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants;
import frc.utils.DriverStationUtils;
import frc.utils.applicationsutils.CMDHandler;
import frc.utils.dashboard.LoggedTableBoolean;

class BatteryLimiter extends Command {

    private final LoggedTableBoolean isBatteryLow;

    private final LinearFilter voltageFilter;

    private double showedMessageTime;

    public BatteryLimiter() {
        this.isBatteryLow = new LoggedTableBoolean("Battery", "is low", false);
        this.voltageFilter = LinearFilter.movingAverage(BatteryConstants.NUMBER_OF_VALUES_IN_AVERAGE);
        this.showedMessageTime = 0;

        if (RobotConstants.ROBOT_TYPE.isSimulation()) {
            CMDHandler.runPythonClass("battery_message", "127.0.0.1");
        }
    }

    private void showBatteryMessage() {
        if (!isBatteryLow.get()) {
            isBatteryLow.set(true);
            showedMessageTime = Timer.getFPGATimestamp();
        }
    }

    private boolean passedMessageStartUpTime() {
        return Timer.getFPGATimestamp() - showedMessageTime > BatteryConstants.MESSAGE_STARTUP_TIME;
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
            Battery.reportLowBatteryToLog();
            showBatteryMessage();
            if (!DriverStationUtils.isGame() && RobotConstants.ENABLE_BATTERY_LIMITER && passedMessageStartUpTime()) {
                throw new java.lang.RuntimeException("BATTERY IS LOW");
            }
        }
        else {
            if (isBatteryLow.get()) {
                isBatteryLow.set(false);
            }
        }
    }

}
