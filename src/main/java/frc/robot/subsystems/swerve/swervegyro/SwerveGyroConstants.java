package frc.robot.subsystems.swerve.swervegyro;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.LogPathsConstants;
import frc.robot.subsystems.swerve.SwerveConstants;

public class SwerveGyroConstants {

    public static final String LOG_PATH = SwerveConstants.SWERVE_LOG_PATH + "Gyro/";
    public static final String ALERT_LOG_PATH = LogPathsConstants.ALERT_LOG_PATH + LOG_PATH;

    public static final Rotation3d GYRO_MOUNT_POSITION = new Rotation3d(
            Units.degreesToRadians(180),
            Units.degreesToRadians(0),
            Units.degreesToRadians(0)
    );

}
