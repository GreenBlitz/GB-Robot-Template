package frc.robot.subsystems.swerve.factories.gyro;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import frc.robot.IDs;
import frc.robot.RobotConstants;
import frc.robot.hardware.phoenix6.Pigeon2Wrapper;
import frc.robot.hardware.phoenix6.pigeon.PigeonHandler;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.subsystems.swerve.GyroSignals;
import frc.utils.AngleUnit;
import frc.utils.alerts.Alert;

class Pigeon2GyroBuilder {

	private static final int APPLY_CONFIG_RETRIES = 5;
	private static PigeonHandler PIGEON;

	private static Pigeon2Configuration buildGyroConfig() {
		Pigeon2Configuration gyroConfig = new Pigeon2Configuration();

		gyroConfig.MountPose.MountPoseRoll = 180;
		gyroConfig.MountPose.MountPosePitch = 0;
		gyroConfig.MountPose.MountPoseYaw = 0;

		return gyroConfig;
	}

	static PigeonHandler buildPigeon(String logPath) {
		if (PIGEON == null) {
			Pigeon2Wrapper pigeon2Wrapper = new Pigeon2Wrapper(IDs.SWERVE_PIGEON_2);
			if (!pigeon2Wrapper.applyConfiguration(buildGyroConfig(), APPLY_CONFIG_RETRIES).isOK()) {
				new Alert(Alert.AlertType.ERROR, logPath + "ConfigurationFailAt").report();
			}
			PIGEON = new PigeonHandler(logPath, pigeon2Wrapper);
		}
		return PIGEON;
	}

	static GyroSignals buildSignals(PigeonHandler pigeon2Gyro) {
		return new GyroSignals(
			Phoenix6SignalBuilder.build(pigeon2Gyro.getDevice().getYaw(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.DEGREES)
		);
	}

}