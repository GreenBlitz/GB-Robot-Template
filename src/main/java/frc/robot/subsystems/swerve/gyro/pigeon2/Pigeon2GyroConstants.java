package frc.robot.subsystems.swerve.gyro.pigeon2;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import frc.robot.constants.DeviceIDs;

public class Pigeon2GyroConstants {

    private static final Pigeon2Configuration PIGEON_2_CONFIGURATION = new Pigeon2Configuration();
    static {
        PIGEON_2_CONFIGURATION.MountPose.MountPoseRoll = 180;
        PIGEON_2_CONFIGURATION.MountPose.MountPosePitch = 0;
        PIGEON_2_CONFIGURATION.MountPose.MountPoseYaw = 0;
    }

    public static final Pigeon2GyroConfigObject PIGEON_2_GYRO_CONFIG_OBJECT = new Pigeon2GyroConfigObject(
            DeviceIDs.PIGEON_2_DEVICE_ID,
            PIGEON_2_CONFIGURATION
    );

}
