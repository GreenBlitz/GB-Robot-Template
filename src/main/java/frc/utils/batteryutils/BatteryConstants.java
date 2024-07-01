package frc.utils.batteryutils;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.constants.LogPathsConstants;

class BatteryConstants {

    protected static final String LOG_PATH = "Battery/";
    protected static final String ALERT_LOG_PATH = LogPathsConstants.ALERT_LOG_PATH + LOG_PATH;

    protected static final int POWER_DISTRIBUTION_CAN_ID = 20;
    protected static final PowerDistribution.ModuleType POWER_DISTRIBUTION_TYPE = PowerDistribution.ModuleType.kRev;

}
