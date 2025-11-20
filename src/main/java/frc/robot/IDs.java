package frc.robot;

import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;

public class IDs {

	public static class TalonFXIDs {

		public final static Phoenix6DeviceID FLYWHEEL = new Phoenix6DeviceID(10, BusChain.ROBORIO);
		public final static Phoenix6DeviceID FLYWHEEL_FOLLOWER = new Phoenix6DeviceID(11, BusChain.ROBORIO);
		
		public final static Phoenix6DeviceID HOOD_ID = new Phoenix6DeviceID(20, BusChain.SUPER);

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
