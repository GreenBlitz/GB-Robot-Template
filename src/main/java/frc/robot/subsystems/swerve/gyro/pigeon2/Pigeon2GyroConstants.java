package frc.robot.subsystems.swerve.gyro.pigeon2;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import frc.robot.constants.Ports;
import frc.robot.subsystems.swerve.gyro.SwerveGyroConstants;

public class Pigeon2GyroConstants {

    private static final Pigeon2Configuration PIGEON_2_CONFIGURATION = new Pigeon2Configuration();
    static {
        PIGEON_2_CONFIGURATION.MountPose.MountPoseRoll = SwerveGyroConstants.GYRO_MOUNT_POSITION_DEGREES.getX();
        PIGEON_2_CONFIGURATION.MountPose.MountPosePitch = SwerveGyroConstants.GYRO_MOUNT_POSITION_DEGREES.getY();
        PIGEON_2_CONFIGURATION.MountPose.MountPoseYaw = SwerveGyroConstants.GYRO_MOUNT_POSITION_DEGREES.getZ();
    }

    public static final Pigeon2GyroConfigObject PIGEON_2_GYRO_CONFIG_OBJECT = new Pigeon2GyroConfigObject(
            Ports.PIGEON_2_DEVICE_ID,
            PIGEON_2_CONFIGURATION
    );

}
