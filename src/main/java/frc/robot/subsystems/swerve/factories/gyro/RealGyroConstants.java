package frc.robot.subsystems.swerve.factories.gyro;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import frc.robot.constants.IDs;
import frc.robot.hardware.gyro.phoenix6.Pigeon2Wrapper;
import frc.robot.hardware.phoenix6.PhoenixProUtils;
import frc.robot.hardware.signal.phoenix.Phoenix6AngleSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6SignalBuilder;
import frc.utils.AngleUnit;
import frc.utils.alerts.Alert;

class RealGyroConstants {

	private static final int APPLY_CONFIG_RETRIES = 10;

	private static Pigeon2Configuration generateGyroConfig() {
		Pigeon2Configuration gyroConfig = new Pigeon2Configuration();

		gyroConfig.MountPose.MountPoseRoll = 180;
		gyroConfig.MountPose.MountPosePitch = 0;
		gyroConfig.MountPose.MountPoseYaw = 0;

		return gyroConfig;
	}

	public static class GyroCreator {

		private final Pigeon2Wrapper pigeon2Wrapper;
		private final Phoenix6AngleSignal yawSignal;

		public GyroCreator() {
			this.pigeon2Wrapper = new Pigeon2Wrapper(IDs.PIGEON_2_DEVICE_ID);
			this.yawSignal = Phoenix6SignalBuilder.generatePhoenix6Signal(pigeon2Wrapper.getYaw(), 50, AngleUnit.DEGREES);
		}

		public Pigeon2Wrapper getPigeon2WrapperWithConfig(String logPath) {
			if (!PhoenixProUtils.checkWithRetry(() -> pigeon2Wrapper.applyConfiguration(generateGyroConfig(), 5), APPLY_CONFIG_RETRIES).isOK()) {
				new Alert(Alert.AlertType.WARNING, logPath + "ConfigurationFailAt").report();
			}
			return pigeon2Wrapper;
		}

		public Phoenix6AngleSignal getYawSignal() {
			return yawSignal;
		}

	}

	protected static final GyroCreator GYRO = new GyroCreator();

}
