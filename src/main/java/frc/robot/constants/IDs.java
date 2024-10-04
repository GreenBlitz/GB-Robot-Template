package frc.robot.constants;


import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.hardware.motor.sparkmax.SparkMaxDeviceID;
import frc.utils.battery.PowerDistributionDeviceID;

public class IDs {

	public static final PowerDistributionDeviceID POWER_DISTRIBUTION_DEVICE_ID = new PowerDistributionDeviceID(
		20,
		PowerDistribution.ModuleType.kRev
	);


	public static class CANSparkMAXIDs {

		public static final SparkMaxDeviceID FUNNEL = new SparkMaxDeviceID(1, CANSparkLowLevel.MotorType.kBrushless);

	}

}
