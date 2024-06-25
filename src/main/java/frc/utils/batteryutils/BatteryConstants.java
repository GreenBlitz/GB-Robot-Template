package frc.utils.batteryutils;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LogPathsConstants;

class BatteryConstants {

    protected static final String LOG_PATH = "Battery/";

    protected static final String ALERT_LOG_PATH = LogPathsConstants.ALERT_LOG_PATH + LOG_PATH;

    protected static final double DEFAULT_BATTERY_VOLTAGE = 12;

    protected static final double MIN_VOLTAGE_BATTERY = 10.5;

    protected static final int NUMBER_OF_VALUES_TAKEN_IN_AVERAGE = 50;

    protected static final int POWER_DISTRIBUTION_CAN_ID = 20;

    protected static final PowerDistribution.ModuleType POWER_DISTRIBUTION_TYPE = PowerDistribution.ModuleType.kRev;

    protected static PowerDistribution POWER_DISTRIBUTION = new PowerDistribution(
            POWER_DISTRIBUTION_CAN_ID,
            POWER_DISTRIBUTION_TYPE
    );

    protected static final Command BATTERY_LIMITER = new BatteryLimiter().ignoringDisable(true);

}
