package frc.robot.subsystems.swerve.modules.inputs;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class SteerMotorInputs {

    public boolean isSteerMotorConnected = true;// todo - test for each component
    public Rotation2d steerMotorAngle = new Rotation2d();
    public Rotation2d steerMotorVelocity = new Rotation2d();
    public Rotation2d steerMotorAcceleration = new Rotation2d();
    public double steerMotorVoltage = 0;
    public Rotation2d[] odometrySamplesSteerAngle = new Rotation2d[0];

}
