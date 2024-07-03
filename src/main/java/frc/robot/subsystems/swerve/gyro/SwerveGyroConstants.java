package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.constants.LogPathsConstants;
import frc.robot.subsystems.swerve.SwerveConstants;

public class SwerveGyroConstants {

    public static final String LOG_PATH = SwerveConstants.SWERVE_LOG_PATH + "Gyro/";
    public static final String ALERT_LOG_PATH = LogPathsConstants.ALERT_LOG_PATH + LOG_PATH;

    public static final Rotation3d GYRO_MOUNT_POSITION_DEGREES = new Rotation3d(180, 0, 0);

}
