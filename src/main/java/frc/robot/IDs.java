package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.rev.motors.SparkMaxDeviceID;
import frc.utils.battery.PowerDistributionDeviceID;

public class IDs {

	public static final PowerDistributionDeviceID POWER_DISTRIBUTION = new PowerDistributionDeviceID(20, PowerDistribution.ModuleType.kRev);

	public static final Phoenix6DeviceID SWERVE_PIGEON_2 = new Phoenix6DeviceID(0, BusChain.CANIVORE);

	public static class TalonFXIDs {

		public static final Phoenix6DeviceID SWERVE_FRONT_LEFT_STEER = new Phoenix6DeviceID(0, BusChain.CANIVORE);

		public static final Phoenix6DeviceID SWERVE_FRONT_LEFT_DRIVE = new Phoenix6DeviceID(1, BusChain.CANIVORE);

		public static final Phoenix6DeviceID SWERVE_FRONT_RIGHT_STEER = new Phoenix6DeviceID(2, BusChain.CANIVORE);

		public static final Phoenix6DeviceID SWERVE_FRONT_RIGHT_DRIVE = new Phoenix6DeviceID(3, BusChain.CANIVORE);

		public static final Phoenix6DeviceID SWERVE_BACK_LEFT_STEER = new Phoenix6DeviceID(4, BusChain.CANIVORE);

		public static final Phoenix6DeviceID SWERVE_BACK_LEFT_DRIVE = new Phoenix6DeviceID(5, BusChain.CANIVORE);

		public static final Phoenix6DeviceID SWERVE_BACK_RIGHT_STEER = new Phoenix6DeviceID(6, BusChain.CANIVORE);

		public static final Phoenix6DeviceID SWERVE_BACK_RIGHT_DRIVE = new Phoenix6DeviceID(7, BusChain.CANIVORE);

		public static final Phoenix6DeviceID ELEVATOR_RIGHT = new Phoenix6DeviceID(10, BusChain.ROBORIO);

		public static final Phoenix6DeviceID ELEVATOR_LEFT = new Phoenix6DeviceID(11, BusChain.ROBORIO);

		public static final Phoenix6DeviceID ARM = new Phoenix6DeviceID(20, BusChain.ROBORIO);

	}

	public static class SparkMAXIDs {

		public static final SparkMaxDeviceID END_EFFECTOR = new SparkMaxDeviceID(5);

	}

	public static class CANCodersIDs {

		public static final Phoenix6DeviceID SWERVE_FRONT_LEFT = new Phoenix6DeviceID(0, BusChain.CANIVORE);

		public static final Phoenix6DeviceID SWERVE_FRONT_RIGHT = new Phoenix6DeviceID(1, BusChain.CANIVORE);

		public static final Phoenix6DeviceID SWERVE_BACK_LEFT = new Phoenix6DeviceID(2, BusChain.CANIVORE);

		public static final Phoenix6DeviceID SWERVE_BACK_RIGHT = new Phoenix6DeviceID(3, BusChain.CANIVORE);

		public static final Phoenix6DeviceID ARM = new Phoenix6DeviceID(20, BusChain.ROBORIO);

	}

}
