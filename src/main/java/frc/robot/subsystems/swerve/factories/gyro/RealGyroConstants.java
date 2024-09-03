package frc.robot.subsystems.swerve.factories.gyro;

import com.ctre.phoenix6.configs.Pigeon2Configuration;

class RealGyroConstants {

	protected static Pigeon2Configuration generateGyroConfiguration() {
		Pigeon2Configuration configuration = new Pigeon2Configuration();

		configuration.MountPose.MountPoseRoll = 180;
		configuration.MountPose.MountPosePitch = 0;
		configuration.MountPose.MountPoseYaw = 0;

		return configuration;
	}

}
