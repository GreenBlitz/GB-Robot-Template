package frc.robot;

import com.revrobotics.spark.SparkLowLevel;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.rev.motors.SparkMaxDeviceID;

public class IDs {

	public static class TalonFXIDs {

		public final static Phoenix6DeviceID FLYWHEEL = new Phoenix6DeviceID(10, BusChain.ROBORIO);
		public final static Phoenix6DeviceID FLYWHEEL_FOLLOWER = new Phoenix6DeviceID(11, BusChain.ROBORIO);

		public final static Phoenix6DeviceID HOOD = new Phoenix6DeviceID(20, BusChain.ROBORIO);
		

		public static final Phoenix6DeviceID TURRET = new Phoenix6DeviceID(30, BusChain.ROBORIO);

	}

	public static class CANCoderIDs {
	}

	public static class Pigeon2IDs {
	}

	public static class CANdleIDs {
	}

	public static class SparkMAXIDs {
		public final static SparkMaxDeviceID OMNI = new SparkMaxDeviceID(25, SparkLowLevel.MotorType.kBrushless);
	}

	public static class DigitalInputsIDs {
	}

}
