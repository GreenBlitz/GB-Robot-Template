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

	public static final Phoenix6DeviceID PIGEON_2_DEVICE_ID = new Phoenix6DeviceID(0, BusChain.CANIVORE);

	public static class TalonFXIDs {

		public static final Phoenix6DeviceID FRONT_LEFT_STEER_MOTOR = new Phoenix6DeviceID(0, BusChain.CANIVORE);

		public static final Phoenix6DeviceID FRONT_LEFT_DRIVE_MOTOR = new Phoenix6DeviceID(1, BusChain.CANIVORE);

		public static final Phoenix6DeviceID FRONT_RIGHT_STEER_MOTOR = new Phoenix6DeviceID(2, BusChain.CANIVORE);

		public static final Phoenix6DeviceID FRONT_RIGHT_DRIVE_MOTOR = new Phoenix6DeviceID(3, BusChain.CANIVORE);

		public static final Phoenix6DeviceID BACK_LEFT_STEER_MOTOR = new Phoenix6DeviceID(4, BusChain.CANIVORE);

		public static final Phoenix6DeviceID BACK_LEFT_DRIVE_MOTOR = new Phoenix6DeviceID(5, BusChain.CANIVORE);

		public static final Phoenix6DeviceID BACK_RIGHT_STEER_MOTOR = new Phoenix6DeviceID(6, BusChain.CANIVORE);

		public static final Phoenix6DeviceID BACK_RIGHT_DRIVE_MOTOR = new Phoenix6DeviceID(7, BusChain.CANIVORE);

	}

	public static class CANCodersIDs {

		public static final Phoenix6DeviceID FRONT_LEFT_ENCODER = new Phoenix6DeviceID(0, BusChain.CANIVORE);

		public static final Phoenix6DeviceID FRONT_RIGHT_ENCODER = new Phoenix6DeviceID(1, BusChain.CANIVORE);

		public static final Phoenix6DeviceID BACK_LEFT_ENCODER = new Phoenix6DeviceID(2, BusChain.CANIVORE);

		public static final Phoenix6DeviceID BACK_RIGHT_ENCODER = new Phoenix6DeviceID(3, BusChain.CANIVORE);

	}

}
