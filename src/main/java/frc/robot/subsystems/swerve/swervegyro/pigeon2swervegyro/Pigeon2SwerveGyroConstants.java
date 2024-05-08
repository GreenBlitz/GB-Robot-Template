package frc.robot.subsystems.swerve.swervegyro.pigeon2swervegyro;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.swervegyro.SwerveGyroConstants;

class Pigeon2SwerveGyroConstants {

    protected static final Pigeon2Configuration PIGEON_2_CONFIGURATION = new Pigeon2Configuration();

    static {
        PIGEON_2_CONFIGURATION.MountPose.MountPoseRoll = Units.radiansToDegrees(SwerveGyroConstants.GYRO_MOUNT_POSITION.getX());
        PIGEON_2_CONFIGURATION.MountPose.MountPosePitch = Units.radiansToDegrees(SwerveGyroConstants.GYRO_MOUNT_POSITION.getY());
        PIGEON_2_CONFIGURATION.MountPose.MountPoseYaw = Units.radiansToDegrees(SwerveGyroConstants.GYRO_MOUNT_POSITION.getZ());
    }
}
