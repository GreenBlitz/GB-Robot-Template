package frc.robot.subsystems.swerve.factories.gyro;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.IDs;
import frc.robot.hardware.gyro.phoenix6.Pigeon2Gyro;
import frc.robot.hardware.gyro.phoenix6.Pigeon2Wrapper;
import frc.robot.hardware.signal.phoenix.Phoenix6AngleSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6SignalBuilder;
import frc.robot.subsystems.swerve.GyroStuff;
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

	protected static GyroStuff generateGyroStuff(String logPath) {
		Pigeon2Wrapper pigeon2Wrapper = new Pigeon2Wrapper(IDs.PIGEON_2);
		if (!pigeon2Wrapper.applyConfiguration(generateGyroConfig(), APPLY_CONFIG_RETRIES).isOK()) {
			new Alert(Alert.AlertType.ERROR, logPath + "ConfigurationFailAt").report();
		}

		Phoenix6AngleSignal yawSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(pigeon2Wrapper.getYaw(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.DEGREES);

		return new GyroStuff(logPath, new Pigeon2Gyro(logPath, pigeon2Wrapper), yawSignal);
	}

}
