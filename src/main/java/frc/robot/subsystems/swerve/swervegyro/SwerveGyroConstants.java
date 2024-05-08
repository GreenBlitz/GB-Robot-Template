package frc.robot.subsystems.swerve.swervegyro;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class SwerveGyroConstants {

    public static final Rotation3d GYRO_MOUNT_POSITION = new Rotation3d(
            Units.degreesToRadians(180),
            Units.degreesToRadians(0),
            Units.degreesToRadians(0)
    );

}
