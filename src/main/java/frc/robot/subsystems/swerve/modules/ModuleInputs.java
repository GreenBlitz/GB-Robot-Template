package frc.robot.subsystems.swerve.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ModuleInputs {

    public double steerEncoderAngleDegrees = 0;
    public double steerEncoderVelocity = 0;
    public double steerEncoderVoltage = 0;

    public Rotation2d steerMotorAngle = new Rotation2d();
    public Rotation2d steerMotorVelocity = new Rotation2d();
    public Rotation2d steerMotorAcceleration = new Rotation2d();
    public double steerMotorVoltage = 0;
    public double[] odometryUpdatesSteerAngleDegrees = new double[0];

    public Rotation2d driveMotorDistance = new Rotation2d();
    public Rotation2d driveMotorVelocity = new Rotation2d();
    public Rotation2d driveMotorAcceleration = new Rotation2d();
    public double driveMotorCurrent = 0;
    public double driveMotorVoltage = 0;
    public double[] odometryUpdatesDriveDistanceDegrees = new double[0];

}
