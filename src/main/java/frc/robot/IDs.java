package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.utils.battery.PowerDistributionDeviceID;

public class IDs {

	public static final PowerDistributionDeviceID POWER_DISTRIBUTION_DEVICE_ID = new PowerDistributionDeviceID(
		-1,
		PowerDistribution.ModuleType.kRev
	);

	public static class TalonFXIDs{
		public static final Phoenix6DeviceID ARM_DEVICE_ID = new Phoenix6DeviceID(2); //ToDo: change before merge


	}

}
