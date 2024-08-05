package frc.utils.battery;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.IPConstants;
import frc.utils.CMDHandler;
import frc.robot.constants.GlobalConstants;
import frc.utils.DriverStationUtils;
import frc.utils.dashboard.LoggedTableBoolean;
import org.littletonrobotics.junction.Logger;

import java.nio.file.Path;

class BatteryLimiter extends Command {

    private static final int NUMBER_OF_SAMPLES_TAKEN_IN_AVERAGE = 50;
    private static final double MESSAGE_STARTUP_TIME = 0.2; // 2 python cycles

    private final LoggedTableBoolean isBatteryLow;
    private final LinearFilter voltageFilter;
    private double showedMessageTime;

    public BatteryLimiter() {
        this.isBatteryLow = new LoggedTableBoolean("Battery", "is low", false);
        this.voltageFilter = LinearFilter.movingAverage(NUMBER_OF_SAMPLES_TAKEN_IN_AVERAGE);
        this.showedMessageTime = Timer.getFPGATimestamp();

        if (Robot.ROBOT_TYPE.isSimulation()) {
            CMDHandler.runPythonClass(Path.of("BatteryMessage"), IPConstants.SIMULATION_IP);
        }
    }

    private void showBatteryMessage() {
        if (!isBatteryLow.get()) {
            isBatteryLow.set(true);
            showedMessageTime = Timer.getFPGATimestamp();
        }
    }

    private boolean passedMessageStartUpTime() {
        return Timer.getFPGATimestamp() - showedMessageTime > MESSAGE_STARTUP_TIME;
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
            if (GlobalConstants.ENABLE_BATTERY_LIMITER && !DriverStationUtils.isMatch() && passedMessageStartUpTime()) {
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
