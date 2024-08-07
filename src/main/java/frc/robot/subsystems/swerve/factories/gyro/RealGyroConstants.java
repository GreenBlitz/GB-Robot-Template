package frc.robot.subsystems.swerve.factories.gyro;

import com.ctre.phoenix6.configs.Pigeon2Configuration;

class RealGyroConstants {

	protected static final Pigeon2Configuration PIGEON_2_CONFIGURATION = new Pigeon2Configuration();
	static {
		PIGEON_2_CONFIGURATION.MountPose.MountPoseRoll = 180;
		PIGEON_2_CONFIGURATION.MountPose.MountPosePitch = 0;
		PIGEON_2_CONFIGURATION.MountPose.MountPoseYaw = 0;
	}

}
