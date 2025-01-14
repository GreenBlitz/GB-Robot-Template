package frc.robot.subsystems.swerve.factories.gyro;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import frc.robot.IDs;
import frc.robot.RobotConstants;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.phoenix6.gyro.Pigeon2Gyro;
import frc.robot.hardware.phoenix6.gyro.Pigeon2Wrapper;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.subsystems.swerve.GyroSignals;
import frc.utils.AngleUnit;
import frc.utils.alerts.Alert;

class RealGyroConstants {

	private static final int APPLY_CONFIG_RETRIES = 5;

	private static Pigeon2Configuration generateGyroConfig() {
		Pigeon2Configuration gyroConfig = new Pigeon2Configuration();

		gyroConfig.MountPose.MountPoseRoll = 180;
		gyroConfig.MountPose.MountPosePitch = 0;
		gyroConfig.MountPose.MountPoseYaw = 0;

		return gyroConfig;
	}

	protected static IGyro generateGyro(String logPath) {
		Pigeon2Wrapper pigeon2Wrapper = new Pigeon2Wrapper(IDs.PIGEON_2_DEVICE_ID);
		if (!pigeon2Wrapper.applyConfiguration(generateGyroConfig(), APPLY_CONFIG_RETRIES).isOK()) {
			new Alert(Alert.AlertType.ERROR, logPath + "ConfigurationFailAt").report();
		}

		return new Pigeon2Gyro(logPath, pigeon2Wrapper);
	}

	protected static GyroSignals generateSignals(Pigeon2Gyro pigeon2Gyro) {
		return new GyroSignals(
			Phoenix6SignalBuilder
				.generatePhoenix6Signal(pigeon2Gyro.getDevice().getYaw(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.DEGREES)
		);
	}

}
