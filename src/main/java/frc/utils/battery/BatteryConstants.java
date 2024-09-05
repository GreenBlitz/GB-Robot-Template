package frc.utils.battery;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.constants.LogPaths;

class BatteryConstants {

	protected static final String LOG_PATH = "Battery/";
	protected static final String ALERT_LOG_PATH = LogPaths.ALERT_LOG_PATH + LOG_PATH;

	protected static final PowerDistributionDeviceID POWER_DISTRIBUTION_DEVICE_ID = new PowerDistributionDeviceID(
		20,
		PowerDistribution.ModuleType.kRev
	);
	protected static final int POWER_DISTRIBUTION_CAN_ID = 20;
	protected static final PowerDistribution.ModuleType POWER_DISTRIBUTION_TYPE = PowerDistribution.ModuleType.kRev;

}
