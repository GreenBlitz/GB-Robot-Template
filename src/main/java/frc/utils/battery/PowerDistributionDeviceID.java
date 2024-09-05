package frc.utils.battery;

import edu.wpi.first.wpilibj.PowerDistribution;

public record PowerDistributionDeviceID(int id, PowerDistribution.ModuleType type) {

	public PowerDistributionDeviceID(int id) {
		this(id, BatteryConstants.DEFAULT_MODULE_TYPE);
	}

}
