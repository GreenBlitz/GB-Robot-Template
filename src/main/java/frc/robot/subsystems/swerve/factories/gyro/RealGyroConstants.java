package frc.robot.subsystems.swerve.factories.gyro;

import com.ctre.phoenix6.configs.Pigeon2Configuration;

class RealGyroConstants {

	protected static Pigeon2Configuration generateGyroConfig() {
		Pigeon2Configuration gyroConfig = new Pigeon2Configuration();

		gyroConfig.MountPose.MountPoseRoll = 180;
		gyroConfig.MountPose.MountPosePitch = 0;
		gyroConfig.MountPose.MountPoseYaw = 0;

		return gyroConfig;
	}

}
