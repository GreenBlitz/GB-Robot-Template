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

	public static class CANSparkMAXs {

		public static final SparkMaxDeviceID FUNNEL = new SparkMaxDeviceID(52, CANSparkLowLevel.MotorType.kBrushless);

		public static final SparkMaxDeviceID INTAKE = new SparkMaxDeviceID(3, CANSparkLowLevel.MotorType.kBrushless);

		public static final SparkMaxDeviceID ELBOW = new SparkMaxDeviceID(5, CANSparkLowLevel.MotorType.kBrushless);

		public static final SparkMaxDeviceID ROLLER = new SparkMaxDeviceID(22, CANSparkLowLevel.MotorType.kBrushless);

	}

	public static class TalonFXs {

		public static final Phoenix6DeviceID RIGHT_FLYWHEEL = new Phoenix6DeviceID(16, BusChain.ROBORIO);

		public static final Phoenix6DeviceID LEFT_FLYWHEEL = new Phoenix6DeviceID(22, BusChain.ROBORIO);

		public static final Phoenix6DeviceID PIVOT = new Phoenix6DeviceID(11, BusChain.CANIVORE);

	}

}
