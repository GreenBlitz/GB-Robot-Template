package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.utils.battery.PowerDistributionDeviceID;

public class IDs {

	public static final PowerDistributionDeviceID POWER_DISTRIBUTION_DEVICE_ID = new PowerDistributionDeviceID(
		-1,
		PowerDistribution.ModuleType.kRev
	);

	public static class Phoenix6IDs {
		public static final Phoenix6DeviceID ELEVATOR_FIRST_MOTOR_ID = new Phoenix6DeviceID(0, BusChain.ROBORIO);

		public static final Phoenix6DeviceID ELEVATOR_SECOND_MOTOR_ID = new Phoenix6DeviceID(1, BusChain.ROBORIO);
	}

}
