package frc.robot.subsystems.swerve.mk4iswerve;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Phoenix6Constants;
import frc.robot.constants.Ports;
import frc.robot.subsystems.Gyro.GyroConstants;
import frc.robot.subsystems.Gyro.pigeon2.Pigeon2GyroConfigObject;

public class MK4ISwerveConstants {

    public static final Pigeon2Configuration PIGEON_2_CONFIGURATION = new Pigeon2Configuration();

    static {
        PIGEON_2_CONFIGURATION.MountPose.MountPoseRoll = Units.radiansToDegrees(GyroConstants.GYRO_MOUNT_POSITION.getX());
        PIGEON_2_CONFIGURATION.MountPose.MountPosePitch = Units.radiansToDegrees(GyroConstants.GYRO_MOUNT_POSITION.getY());
        PIGEON_2_CONFIGURATION.MountPose.MountPoseYaw = Units.radiansToDegrees(GyroConstants.GYRO_MOUNT_POSITION.getZ());
    }

    protected static final Pigeon2GyroConfigObject PIGEON_2_GYRO_CONFIG_OBJECT = new Pigeon2GyroConfigObject(
            Ports.PIGEON_2_ID,
            Phoenix6Constants.CANIVORE_NAME
    );

}


