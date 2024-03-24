package frc.robot.subsystems.Gyro;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Phoenix6Constants;
import frc.robot.constants.Ports;

public class GyroConstants {

    private static final Rotation3d GYRO_MOUNT_POSITION = new Rotation3d(
            Units.degreesToRadians(0),
            Units.degreesToRadians(0),
            Units.degreesToRadians(0)
    );

    public static final Pigeon2Configuration PIGEON_2_CONFIGURATION = new Pigeon2Configuration();
    static {
        PIGEON_2_CONFIGURATION.MountPose.MountPoseRoll = Units.radiansToDegrees(GYRO_MOUNT_POSITION.getX());
        PIGEON_2_CONFIGURATION.MountPose.MountPosePitch = Units.radiansToDegrees(GYRO_MOUNT_POSITION.getY());
        PIGEON_2_CONFIGURATION.MountPose.MountPoseYaw = Units.radiansToDegrees(GYRO_MOUNT_POSITION.getZ());
    }

    public static final Pigeon2GyroConfigObject PIGEON_2_GYRO_CONFIG_OBJECT = new Pigeon2GyroConfigObject(
            Ports.PIGEON_2_ID,
            Phoenix6Constants.CANIVORE_NAME
    );

}
