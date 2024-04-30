package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class GyroConstants {

    public static final Rotation3d GYRO_MOUNT_POSITION = new Rotation3d(
            Units.degreesToRadians(180),
            Units.degreesToRadians(0),
            Units.degreesToRadians(0)
    );

}
