package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.rev.motors.SparkMaxDeviceID;
import frc.utils.battery.PowerDistributionDeviceID;

public class IDs {

	public static final PowerDistributionDeviceID POWER_DISTRIBUTION = new PowerDistributionDeviceID(1, PowerDistribution.ModuleType.kRev);

	public static class TalonFXIDs {

		public static final Phoenix6DeviceID SWERVE_FRONT_LEFT_STEER_MOTOR = new Phoenix6DeviceID(0, BusChain.CANIVORE);

		public static final Phoenix6DeviceID SWERVE_FRONT_LEFT_DRIVE_MOTOR = new Phoenix6DeviceID(1, BusChain.CANIVORE);

		public static final Phoenix6DeviceID SWERVE_FRONT_RIGHT_STEER_MOTOR = new Phoenix6DeviceID(2, BusChain.CANIVORE);

		public static final Phoenix6DeviceID SWERVE_FRONT_RIGHT_DRIVE_MOTOR = new Phoenix6DeviceID(3, BusChain.CANIVORE);

		public static final Phoenix6DeviceID SWERVE_BACK_LEFT_STEER_MOTOR = new Phoenix6DeviceID(4, BusChain.CANIVORE);

		public static final Phoenix6DeviceID SWERVE_BACK_LEFT_DRIVE_MOTOR = new Phoenix6DeviceID(5, BusChain.CANIVORE);

		public static final Phoenix6DeviceID SWERVE_BACK_RIGHT_STEER_MOTOR = new Phoenix6DeviceID(6, BusChain.CANIVORE);

		public static final Phoenix6DeviceID SWERVE_BACK_RIGHT_DRIVE_MOTOR = new Phoenix6DeviceID(7, BusChain.CANIVORE);

		public static final Phoenix6DeviceID ELEVATOR_FIRST_MOTOR = new Phoenix6DeviceID(10, BusChain.ROBORIO);

		public static final Phoenix6DeviceID ELEVATOR_SECOND_MOTOR = new Phoenix6DeviceID(11, BusChain.ROBORIO);

		public static final Phoenix6DeviceID ARM_MOTOR = new Phoenix6DeviceID(20, BusChain.CANIVORE);

	}

	public static class SparkMAXIDs {

		public static final SparkMaxDeviceID END_EFFECTOR_ROLLER = new SparkMaxDeviceID(-1);

	}

	public static class CANCodersIDs {

		public static final Phoenix6DeviceID SWERVE_FRONT_LEFT_ENCODER = new Phoenix6DeviceID(0, BusChain.CANIVORE);

		public static final Phoenix6DeviceID SWERVE_FRONT_RIGHT_ENCODER = new Phoenix6DeviceID(1, BusChain.CANIVORE);

		public static final Phoenix6DeviceID SWERVE_BACK_LEFT_ENCODER = new Phoenix6DeviceID(2, BusChain.CANIVORE);

		public static final Phoenix6DeviceID SWERVE_BACK_RIGHT_ENCODER = new Phoenix6DeviceID(3, BusChain.CANIVORE);

		public static final Phoenix6DeviceID ARM_ENCODER = new Phoenix6DeviceID(20, BusChain.CANIVORE);

	}

}
