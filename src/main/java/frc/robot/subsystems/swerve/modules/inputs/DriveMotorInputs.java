package frc.robot.subsystems.swerve.modules.inputs;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class DriveMotorInputs {

    public boolean isConnected = true;// todo - test for each component
    public double distanceMeters = 0;
    public double velocityMeters = 0;
    public Rotation2d angleWithoutCoupling = new Rotation2d();
    public Rotation2d velocityWithoutCoupling = new Rotation2d();
    public Rotation2d acceleration = new Rotation2d();
    public double current = 0;
    public double voltage = 0;
    public Rotation2d[] distanceOdometrySamples = new Rotation2d[0];

}
