package frc.robot.constants;


import edu.wpi.first.wpilibj.PowerDistribution;
import frc.utils.battery.PowerDistributionDeviceID;
import frc.robot.hardware.motor.sparkmax.SparkMaxDeviceID;


public class IDs {

	public static final PowerDistributionDeviceID POWER_DISTRIBUTION_DEVICE_ID = new PowerDistributionDeviceID(
		20,
		PowerDistribution.ModuleType.kRev
	);

	public static class SparkMaxIDs {

		public static final SparkMaxDeviceID ELEVATOR_ROLLER = new SparkMaxDeviceID(10);

	}

}
