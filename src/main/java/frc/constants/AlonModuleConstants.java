package frc.constants;

import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;

public class AlonModuleConstants {

	public static final Phoenix6DeviceID steerId = new Phoenix6DeviceID(11, BusChain.ROBORIO);
	public static final Phoenix6DeviceID linearId = new Phoenix6DeviceID(10, BusChain.ROBORIO);
	public static final double steerKp = 4;
	public static final double canCoderMagnetOffset = 67;
	public static final double steerGearRatio = 12.8;
	public static final double linearKp = 2;
	public static final double linearGearRatio = 1;
	public static final double moduleAlonStickTolerance = 0.1;
	public static final double steerMomentOfInertiaKgM2 = 0.0000000019;
	public static final double linearMomentOfInertiaKgM2 = 0.0000000019;

}
