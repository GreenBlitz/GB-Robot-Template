package frc.robot.subsystems.swerve.mk4iswerve;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.gyro.GyroConstants;

public class MK4ISwerveConstants {

    public static final Pigeon2Configuration PIGEON_2_CONFIGURATION = new Pigeon2Configuration();
    static {
        PIGEON_2_CONFIGURATION.MountPose.MountPoseRoll = Units.radiansToDegrees(GyroConstants.GYRO_MOUNT_POSITION.getX());
        PIGEON_2_CONFIGURATION.MountPose.MountPosePitch = Units.radiansToDegrees(GyroConstants.GYRO_MOUNT_POSITION.getY());
        PIGEON_2_CONFIGURATION.MountPose.MountPoseYaw = Units.radiansToDegrees(GyroConstants.GYRO_MOUNT_POSITION.getZ());
    }

}


