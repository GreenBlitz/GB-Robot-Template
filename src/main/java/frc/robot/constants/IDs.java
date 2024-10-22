package frc.robot.constants;

import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.hardware.motor.sparkmax.SparkMaxDeviceID;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.utils.battery.PowerDistributionDeviceID;

public class IDs {

	public static final PowerDistributionDeviceID POWER_DISTRIBUTION_DEVICE_ID = new PowerDistributionDeviceID(
		20,
		PowerDistribution.ModuleType.kRev
	);

	public static final Phoenix6DeviceID PIGEON_2_DEVICE_ID = new Phoenix6DeviceID(0, BusChain.CANIVORE);

	public static class TalonSRXs {

		public static final int SOLENOID = 55;

		public static final int WRIST = 11;

	}

	public static class CANSparkMAXs {

		public static final SparkMaxDeviceID FUNNEL = new SparkMaxDeviceID(52, CANSparkLowLevel.MotorType.kBrushless);

		public static final SparkMaxDeviceID INTAKE = new SparkMaxDeviceID(3, CANSparkLowLevel.MotorType.kBrushless);

		public static final SparkMaxDeviceID ELBOW = new SparkMaxDeviceID(5, CANSparkLowLevel.MotorType.kBrushless);

		public static final SparkMaxDeviceID ROLLER = new SparkMaxDeviceID(22, CANSparkLowLevel.MotorType.kBrushless);

	}

	public static class TalonFXIDs {

		public static final Phoenix6DeviceID FRONT_LEFT_STEER_MOTOR = new Phoenix6DeviceID(0, BusChain.CANIVORE);

		public static final Phoenix6DeviceID FRONT_LEFT_DRIVE_MOTOR = new Phoenix6DeviceID(1, BusChain.CANIVORE);

		public static final Phoenix6DeviceID FRONT_RIGHT_STEER_MOTOR = new Phoenix6DeviceID(2, BusChain.CANIVORE);

		public static final Phoenix6DeviceID FRONT_RIGHT_DRIVE_MOTOR = new Phoenix6DeviceID(3, BusChain.CANIVORE);

		public static final Phoenix6DeviceID BACK_LEFT_STEER_MOTOR = new Phoenix6DeviceID(4, BusChain.CANIVORE);

		public static final Phoenix6DeviceID BACK_LEFT_DRIVE_MOTOR = new Phoenix6DeviceID(5, BusChain.CANIVORE);

		public static final Phoenix6DeviceID BACK_RIGHT_STEER_MOTOR = new Phoenix6DeviceID(6, BusChain.CANIVORE);

		public static final Phoenix6DeviceID BACK_RIGHT_DRIVE_MOTOR = new Phoenix6DeviceID(7, BusChain.CANIVORE);

		public static final Phoenix6DeviceID RIGHT_FLYWHEEL = new Phoenix6DeviceID(16, BusChain.ROBORIO);

		public static final Phoenix6DeviceID LEFT_FLYWHEEL = new Phoenix6DeviceID(22, BusChain.ROBORIO);

		public static final Phoenix6DeviceID PIVOT = new Phoenix6DeviceID(11, BusChain.CANIVORE);

		public static final Phoenix6DeviceID LIFTER = new Phoenix6DeviceID(12, BusChain.CANIVORE);

	}

	public static class CANCodersIDs {

		public static final Phoenix6DeviceID FRONT_LEFT_ENCODER = new Phoenix6DeviceID(0, BusChain.CANIVORE);

		public static final Phoenix6DeviceID FRONT_RIGHT_ENCODER = new Phoenix6DeviceID(1, BusChain.CANIVORE);

		public static final Phoenix6DeviceID BACK_LEFT_ENCODER = new Phoenix6DeviceID(2, BusChain.CANIVORE);

		public static final Phoenix6DeviceID BACK_RIGHT_ENCODER = new Phoenix6DeviceID(3, BusChain.CANIVORE);

	}

	public static class DigitalInputsIDs {

		public static final int LIFTER_LIMIT_SWITCH = 9;

	}

}
