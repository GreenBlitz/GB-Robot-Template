package frc.robot.subsystems.swerve.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ModuleInputs {

    public double steerEncoderAngleDegrees = 0;
    public double steerEncoderVelocity = 0;
    public double steerEncoderVoltage = 0;

    public Rotation2d steerMotorAngle = 0;
    public Rotation2d steerMotorVelocity = 0;
    public double steerMotorVoltage = 0;
    public double[] odometryUpdatesSteerAngleDegrees = new double[0];

    public Rotation2d driveDistance = 0;
    public Rotation2d driveVelocityPerSecond = 0;
    public double driveCurrent = 0;
    public double driveVoltage = 0;
    public double[] odometryUpdatesDriveDistanceDegrees = new double[0];

}
