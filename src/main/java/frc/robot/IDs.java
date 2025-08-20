package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.utils.battery.PowerDistributionDeviceID;

public class IDs {

	public static final PowerDistributionDeviceID POWER_DISTRIBUTION = new PowerDistributionDeviceID(1, PowerDistribution.ModuleType.kRev);

	public static class TalonFXIDs {

		public static final Phoenix6DeviceID SWERVE_FRONT_LEFT_STEER = new Phoenix6DeviceID(0, BusChain.ROBORIO);

		public static final Phoenix6DeviceID SWERVE_FRONT_LEFT_DRIVE = new Phoenix6DeviceID(1, BusChain.ROBORIO);

		public static final Phoenix6DeviceID SWERVE_FRONT_RIGHT_STEER = new Phoenix6DeviceID(2, BusChain.ROBORIO);

		public static final Phoenix6DeviceID SWERVE_FRONT_RIGHT_DRIVE = new Phoenix6DeviceID(3, BusChain.ROBORIO);

		public static final Phoenix6DeviceID SWERVE_BACK_LEFT_STEER = new Phoenix6DeviceID(4, BusChain.ROBORIO);

		public static final Phoenix6DeviceID SWERVE_BACK_LEFT_DRIVE = new Phoenix6DeviceID(5, BusChain.ROBORIO);

		public static final Phoenix6DeviceID SWERVE_BACK_RIGHT_STEER = new Phoenix6DeviceID(6, BusChain.ROBORIO);

		public static final Phoenix6DeviceID SWERVE_BACK_RIGHT_DRIVE = new Phoenix6DeviceID(7, BusChain.ROBORIO);

	}

	public static class CANCoderIDs {

		public static final Phoenix6DeviceID SWERVE_FRONT_LEFT = new Phoenix6DeviceID(0, BusChain.ROBORIO);

		public static final Phoenix6DeviceID SWERVE_FRONT_RIGHT = new Phoenix6DeviceID(1, BusChain.ROBORIO);

		public static final Phoenix6DeviceID SWERVE_BACK_LEFT = new Phoenix6DeviceID(2, BusChain.ROBORIO);

		public static final Phoenix6DeviceID SWERVE_BACK_RIGHT = new Phoenix6DeviceID(3, BusChain.ROBORIO);

	}

	public static class Pigeon2IDs {

		public static final Phoenix6DeviceID SWERVE = new Phoenix6DeviceID(0, BusChain.ROBORIO);

	}

	public static class CANdleIDs {
	}

	public static class SparkMAXIDs {
	}

	public static class DigitalInputsIDs {
	}

}
