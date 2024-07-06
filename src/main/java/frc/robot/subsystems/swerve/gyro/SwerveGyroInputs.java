package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class SwerveGyroInputs {

    public boolean connected = true;

    public Rotation2d gyroYaw = new Rotation2d();

    public double accelerationX = 0;
    public double accelerationY = 0;
    public double accelerationZ = 0;

    public Rotation2d[] odometrySamplesYaw = new Rotation2d[0];
    public double[] odometrySamplesTimestamp = new double[0];

}
