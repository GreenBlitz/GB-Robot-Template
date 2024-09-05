package frc.robot.constants;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.utils.battery.PowerDistributionDeviceID;

public class IDs {

    public static final PowerDistributionDeviceID POWER_DISTRIBUTION_DEVICE_ID = new PowerDistributionDeviceID(
            20,
            PowerDistribution.ModuleType.kRev
    );

}
