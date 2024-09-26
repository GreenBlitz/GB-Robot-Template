package frc.robot.constants;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.hardware.motor.sparkmax.SparkMaxDeviceID;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.utils.battery.PowerDistributionDeviceID;

public class IDs {

    public static final PowerDistributionDeviceID POWER_DISTRIBUTION_DEVICE_ID = new PowerDistributionDeviceID(
            20,
            PowerDistribution.ModuleType.kRev
    );

    public static class CANSparkMaxIDs {
        public static final SparkMaxDeviceID INTAKE_ID = new SparkMaxDeviceID(0); //find
    }

    public static class Phoenix6DevicesIDs {
		public static final Phoenix6DeviceID INTAKE_ID = new Phoenix6DeviceID(0);
    }

}
