package frc.robot.subsystems.swerve.modules.moduleinterface;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ModuleInputs {//todo-maybe organize in more inputs packages in the module(steer, drive, encoder, general in module itself)

    public boolean isAtTargetState = true;

    public boolean isEncoderConnected = true;// todo - test for each component
    public Rotation2d encoderAngle = new Rotation2d();
    public Rotation2d encoderVelocity = new Rotation2d();
    public double encoderVoltage = 0;

    public boolean isSteerMotorConnected = true;// todo - test for each component
    public Rotation2d steerMotorAngle = new Rotation2d();
    public Rotation2d steerMotorVelocity = new Rotation2d();
    public Rotation2d steerMotorAcceleration = new Rotation2d();
    public double steerMotorVoltage = 0;
    public Rotation2d[] odometrySamplesSteerAngle = new Rotation2d[0];

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
