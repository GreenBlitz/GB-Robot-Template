package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.rev.motors.SparkMaxDeviceID;
import frc.utils.battery.PowerDistributionDeviceID;

public class IDs {

	public static final PowerDistributionDeviceID POWER_DISTRIBUTION_DEVICE_ID = new PowerDistributionDeviceID(
		1,
		PowerDistribution.ModuleType.kRev
	);

	public static class TalonFXIDs {

		public static final Phoenix6DeviceID ELEVATOR_FIRST_MOTOR_ID = new Phoenix6DeviceID(0, BusChain.ROBORIO);

		public static final Phoenix6DeviceID ELEVATOR_SECOND_MOTOR_ID = new Phoenix6DeviceID(1, BusChain.ROBORIO);

	}

	public static class SparkMAXIDs {

		public static final SparkMaxDeviceID END_EFFECTOR_ROLLER_ID = new SparkMaxDeviceID(-1);

	}

}
