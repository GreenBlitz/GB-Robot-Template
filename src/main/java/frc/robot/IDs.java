package frc.robot;

import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;

public class IDs {

	public static class TalonFXIDs {

		public final static Phoenix6DeviceID FLY_WHEEL_ID = new Phoenix6DeviceID(1, BusChain.ROBORIO);
		public final static Phoenix6DeviceID FLY_WHEEL_FOLLOWER_ID = new Phoenix6DeviceID(2, BusChain.ROBORIO);

	}

	public static class CANCoderIDs {
	}

	public static class Pigeon2IDs {
	}

	public static class CANdleIDs {
	}

	public static class SparkMAXIDs {
	}

	public static class DigitalInputsIDs {
	}

}
