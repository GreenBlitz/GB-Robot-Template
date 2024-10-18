package frc.robot.constants;


import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.motor.sparkmax.SparkMaxDeviceID;
import frc.utils.battery.PowerDistributionDeviceID;

public class IDs {

	public static final PowerDistributionDeviceID POWER_DISTRIBUTION_DEVICE_ID = new PowerDistributionDeviceID(
		20,
		PowerDistribution.ModuleType.kRev
	);

	public static final Phoenix6DeviceID PIGEON_2 = new Phoenix6DeviceID(0, BusChain.CANIVORE);

	public static class TalonSRXIDs {

		public static final int ELEVATOR_ROLLER = 16;

	}

	public static class TalonFXIDs {

		public static final Phoenix6DeviceID FRONT_LEFT_STEER = new Phoenix6DeviceID(0, BusChain.CANIVORE);

		public static final Phoenix6DeviceID FRONT_LEFT_DRIVE = new Phoenix6DeviceID(1, BusChain.CANIVORE);

		public static final Phoenix6DeviceID FRONT_RIGHT_STEER = new Phoenix6DeviceID(2, BusChain.CANIVORE);

		public static final Phoenix6DeviceID FRONT_RIGHT_DRIVE = new Phoenix6DeviceID(3, BusChain.CANIVORE);

		public static final Phoenix6DeviceID BACK_LEFT_STEER = new Phoenix6DeviceID(4, BusChain.CANIVORE);

		public static final Phoenix6DeviceID BACK_LEFT_DRIVE = new Phoenix6DeviceID(5, BusChain.CANIVORE);

		public static final Phoenix6DeviceID BACK_RIGHT_STEER = new Phoenix6DeviceID(6, BusChain.CANIVORE);

		public static final Phoenix6DeviceID BACK_RIGHT_DRIVE = new Phoenix6DeviceID(7, BusChain.CANIVORE);

	}

	public static class CANCodersIDs {

		public static final Phoenix6DeviceID FRONT_LEFT_ENCODER = new Phoenix6DeviceID(0, BusChain.CANIVORE);

		public static final Phoenix6DeviceID FRONT_RIGHT_ENCODER = new Phoenix6DeviceID(1, BusChain.CANIVORE);

		public static final Phoenix6DeviceID BACK_LEFT_ENCODER = new Phoenix6DeviceID(2, BusChain.CANIVORE);

		public static final Phoenix6DeviceID BACK_RIGHT_ENCODER = new Phoenix6DeviceID(3, BusChain.CANIVORE);

	}

	public static class CANSparkMAXIDs {

		public static final SparkMaxDeviceID TOP_FLYWHEEL = new SparkMaxDeviceID(61, CANSparkLowLevel.MotorType.kBrushless);

		public static final SparkMaxDeviceID BOTTOM_FLYWHEEL = new SparkMaxDeviceID(62, CANSparkLowLevel.MotorType.kBrushless);

		public static final SparkMaxDeviceID INTAKE_ROLLER = new SparkMaxDeviceID(22, CANSparkLowLevel.MotorType.kBrushless);

		public static final SparkMaxDeviceID FUNNEL = new SparkMaxDeviceID(11, CANSparkLowLevel.MotorType.kBrushless);

		public static final SparkMaxDeviceID PIVOT = new SparkMaxDeviceID(58, CANSparkLowLevel.MotorType.kBrushless);

		public static final SparkMaxDeviceID ELEVATOR = new SparkMaxDeviceID(60, CANSparkLowLevel.MotorType.kBrushless);

	}

}
