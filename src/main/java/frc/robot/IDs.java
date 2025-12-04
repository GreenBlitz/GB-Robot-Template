package frc.robot;

import com.revrobotics.spark.SparkLowLevel;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.rev.motors.SparkMaxDeviceID;

public class IDs {

	public static class TalonFXIDs {

		public static final Phoenix6DeviceID SWERVE_FRONT_LEFT_STEER = new Phoenix6DeviceID(0, BusChain.ROBORIO);
		public static final Phoenix6DeviceID SWERVE_FRONT_LEFT_DRIVE = new Phoenix6DeviceID(1, BusChain.ROBORIO);
		public static final Phoenix6DeviceID SWERVE_FRONT_RIGHT_STEER = new Phoenix6DeviceID(2, BusChain.ROBORIO);
		public static final Phoenix6DeviceID SWERVE_FRONT_RIGHT_DRIVE = new Phoenix6DeviceID(3, BusChain.ROBORIO);
		public static final Phoenix6DeviceID SWERVE_BACK_LEFT_STEER = new Phoenix6DeviceID(4, BusChain.ROBORIO);
		public static final Phoenix6DeviceID SWERVE_BACK_LEFT_DRIVE = new Phoenix6DeviceID(5, BusChain.ROBORIO);
		public static final Phoenix6DeviceID SWERVE_BACK_RIGHT_STEER = new Phoenix6DeviceID(6, BusChain.ROBORIO);
		public static final Phoenix6DeviceID SWERVE_BACK_RIGHT_DRIVE = new Phoenix6DeviceID(7, BusChain.ROBORIO);

		public final static Phoenix6DeviceID FLYWHEEL = new Phoenix6DeviceID(10, BusChain.ROBORIO);
		public final static Phoenix6DeviceID FLYWHEEL_FOLLOWER = new Phoenix6DeviceID(11, BusChain.ROBORIO);

		public final static Phoenix6DeviceID FOUR_BAR = new Phoenix6DeviceID(12, BusChain.ROBORIO);

		public final static Phoenix6DeviceID HOOD = new Phoenix6DeviceID(20, BusChain.ROBORIO);

		public static final Phoenix6DeviceID TURRET = new Phoenix6DeviceID(30, BusChain.ROBORIO);

	}

	public static class CANCoderIDs {

		public static final Phoenix6DeviceID SWERVE_FRONT_LEFT = new Phoenix6DeviceID(0, BusChain.ROBORIO);

		public static final Phoenix6DeviceID SWERVE_FRONT_RIGHT = new Phoenix6DeviceID(1, BusChain.ROBORIO);

		public static final Phoenix6DeviceID SWERVE_BACK_LEFT = new Phoenix6DeviceID(2, BusChain.ROBORIO);

		public static final Phoenix6DeviceID SWERVE_BACK_RIGHT = new Phoenix6DeviceID(3, BusChain.ROBORIO);

	}

	public static class Pigeon2IDs {
	}

	public static class CANdleIDs {
	}

	public static class SparkMAXIDs {

		public final static SparkMaxDeviceID INTAKE_ROLLERS = new SparkMaxDeviceID(6, SparkLowLevel.MotorType.kBrushless);
		public static final SparkMaxDeviceID BELLY = new SparkMaxDeviceID(35, SparkLowLevel.MotorType.kBrushless);

		public final static SparkMaxDeviceID OMNI = new SparkMaxDeviceID(25, SparkLowLevel.MotorType.kBrushless);

	}

	public static class DigitalInputsIDs {
	}

}
