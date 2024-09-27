package frc.robot.constants;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.utils.battery.PowerDistributionDeviceID;

public class IDs {

	public static final PowerDistributionDeviceID POWER_DISTRIBUTION_DEVICE_ID = new PowerDistributionDeviceID(
		20,
		PowerDistribution.ModuleType.kRev
	);

	public static class TalonFXs {

		public static final Phoenix6DeviceID RIGHT_FLYWHEEL = new Phoenix6DeviceID(16, BusChain.ROBORIO);

		public static final Phoenix6DeviceID LEFT_FLYWHEEL = new Phoenix6DeviceID(22, BusChain.ROBORIO);

	}

}
