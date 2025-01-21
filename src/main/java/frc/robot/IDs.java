package frc.robot;

import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.angleencoder.CANCoderEncoder;
import frc.utils.battery.PowerDistributionDeviceID;

public class IDs {

	public static final PowerDistributionDeviceID POWER_DISTRIBUTION_DEVICE_ID = new PowerDistributionDeviceID(
		-1,
		PowerDistribution.ModuleType.kRev
	);

	public static class CANCodersIDs{
		public static final CoreCANcoder ARM_CAN_CODER = new CoreCANcoder(-1);
	}

	public static class TalonFXIDs {

		public static final Phoenix6DeviceID ARM_DEVICE_ID = new Phoenix6DeviceID(-2); // Todo: change before merge

	}

}
