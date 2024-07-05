package frc.robot.subsystems.swerve.modules.inputs;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class DriveMotorInputs {

    public boolean isDriveMotorConnected = true;// todo - test for each component
    public double driveMotorDistanceMeters = 0;
    public double driveMotorVelocityMeters = 0;
    public Rotation2d driveMotorAngleWithoutCoupling = new Rotation2d();
    public Rotation2d driveMotorVelocityWithoutCoupling = new Rotation2d();
    public Rotation2d driveMotorAcceleration = new Rotation2d();
    public double driveMotorCurrent = 0;
    public double driveMotorVoltage = 0;
    public Rotation2d[] odometrySamplesDriveDistance = new Rotation2d[0];

}
