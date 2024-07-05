package frc.robot.subsystems.swerve.modules.inputs;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class SteerMotorInputs {

    public boolean isConnected = true;// todo - test for each component
    public Rotation2d angle = new Rotation2d();
    public Rotation2d velocity = new Rotation2d();
    public Rotation2d acceleration = new Rotation2d();
    public double voltage = 0;
    public Rotation2d[] odometrySamplesAngle = new Rotation2d[0];

}
