package frc.robot.subsystems.swerve.gyro.pigeon2gyro;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import frc.robot.subsystems.swerve.gyro.SwerveGyroConstants;

class Pigeon2GyroConstants {

    protected static final Pigeon2Configuration PIGEON_2_CONFIGURATION = new Pigeon2Configuration();

    static {
        PIGEON_2_CONFIGURATION.MountPose.MountPoseRoll = SwerveGyroConstants.GYRO_MOUNT_POSITION_DEGREES.getX();
        PIGEON_2_CONFIGURATION.MountPose.MountPosePitch = SwerveGyroConstants.GYRO_MOUNT_POSITION_DEGREES.getY();
        PIGEON_2_CONFIGURATION.MountPose.MountPoseYaw = SwerveGyroConstants.GYRO_MOUNT_POSITION_DEGREES.getZ();
    }
}
