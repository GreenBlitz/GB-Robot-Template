package frc.robot.subsystems.swerve.modules.moduleinterface;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ModuleInputs {//todo-maybe organize in more inputs packages in the module(steer, drive, encoder, general in module itself)

    public boolean allComponentsConnected = true;// todo - make it for each component

    public Rotation2d steerEncoderAngle = new Rotation2d();
    public Rotation2d steerEncoderVelocity = new Rotation2d();
    public double steerEncoderVoltage = 0;

    public Rotation2d steerMotorAngle = new Rotation2d();
    public Rotation2d steerMotorVelocity = new Rotation2d();
    public Rotation2d steerMotorAcceleration = new Rotation2d();
    public double steerMotorVoltage = 0;
    public Rotation2d[] odometryUpdatesSteerAngle = new Rotation2d[0];

    public double driveMotorDistanceMeters = 0;
    public double driveMotorVelocityMeters = 0;
    public Rotation2d driveMotorAngle = new Rotation2d();
    public Rotation2d driveMotorVelocity = new Rotation2d();
    public Rotation2d driveMotorAcceleration = new Rotation2d();
    public double driveMotorCurrent = 0;
    public double driveMotorVoltage = 0;
    public Rotation2d[] odometryUpdatesDriveDistance = new Rotation2d[0];

}
