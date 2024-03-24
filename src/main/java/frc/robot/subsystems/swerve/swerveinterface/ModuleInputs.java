package frc.robot.subsystems.swerve.swerveinterface;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ModuleInputs {

    public double steerEncoderVelocity = 0;
    public double steerEncoderAngleDegrees = 0;
    public double steerEncoderVoltage = 0;

    public double steerAngleDegrees = 0;
    public double steerVelocity = 0;
    public double[] odometryUpdatesSteerAngleDegrees = new double[0];
    public double steerVoltage = 0;

    public double driveVelocityMetersPerSecond = 0;
    public double driveDistanceMeters = 0;
    public double[] odometryUpdatesDriveDistanceMeters = new double[0];
    public double driveCurrent = 0;
    public double driveVoltage = 0;

}
