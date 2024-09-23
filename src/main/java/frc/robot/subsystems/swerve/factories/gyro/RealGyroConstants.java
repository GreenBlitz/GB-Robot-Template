package frc.robot.subsystems.swerve.factories.gyro;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import frc.robot.constants.IDs;
import frc.robot.hardware.gyro.phoenix6.Pigeon2Wrapper;
import frc.robot.hardware.signal.phoenix.Phoenix6AngleSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6SignalBuilder;
import frc.utils.AngleUnit;

class RealGyroConstants {

	private static Pigeon2Configuration generateGyroConfig() {
		Pigeon2Configuration gyroConfig = new Pigeon2Configuration();

		gyroConfig.MountPose.MountPoseRoll = 180;
		gyroConfig.MountPose.MountPosePitch = 0;
		gyroConfig.MountPose.MountPoseYaw = 0;

		return gyroConfig;
	}

	private static final Pigeon2Wrapper pigeon2Wrapper;
	private static final Phoenix6AngleSignal yawSignal;

	static {
		pigeon2Wrapper = new Pigeon2Wrapper(IDs.PIGEON_2_DEVICE_ID);
		pigeon2Wrapper.applyConfiguration(generateGyroConfig(), 5);

		yawSignal = Phoenix6SignalBuilder.generatePhoenix6Signal(pigeon2Wrapper.getYaw(), 50, AngleUnit.DEGREES);
	}

	public static Pigeon2Wrapper getPigeon2Wrapper() {
		return pigeon2Wrapper;
	}

	public static Phoenix6AngleSignal getYawSignal() {
		return yawSignal;
	}

}
