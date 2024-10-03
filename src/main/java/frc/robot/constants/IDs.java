package frc.robot.constants;


import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.hardware.motor.sparkmax.SparkMaxDeviceID;
import frc.utils.battery.PowerDistributionDeviceID;

public class IDs {

	public static final PowerDistributionDeviceID POWER_DISTRIBUTION_DEVICE_ID = new PowerDistributionDeviceID(
		20,
		PowerDistribution.ModuleType.kRev
	);

	public static final class CANSparkMaxIDs {

		public static final SparkMaxDeviceID TOP_FLYWHEEL_MOTOR = new SparkMaxDeviceID(6);
		public static final SparkMaxDeviceID BOTTOM_FLYWHEEL_MOTOR = new SparkMaxDeviceID(7);

	}

}
