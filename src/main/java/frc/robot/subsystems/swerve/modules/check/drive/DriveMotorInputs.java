package frc.robot.subsystems.swerve.modules.check.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class DriveMotorInputs {

    public boolean isConnected = true;
    public double distanceMeters = 0;
    public double velocityMeters = 0;
    public Rotation2d angle = new Rotation2d();
    public Rotation2d velocity = new Rotation2d();
    public Rotation2d acceleration = new Rotation2d();
    public double current = 0;
    public double voltage = 0;
    public Rotation2d[] angleOdometrySamples = new Rotation2d[0];
    public double[] distanceMetersOdometrySamples = new double[0];

}
