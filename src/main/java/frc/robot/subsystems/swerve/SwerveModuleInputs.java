package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class SwerveModuleInputs {
    public double steerAngleDegrees = 0;
    public double[] odometryUpdatesSteerAngleDegrees = new double[0];
    public double steerVoltage = 0;

    public double driveVelocityMetersPerSecond = 0;
    public double driveDistanceMeters = 0;
    public double[] odometryUpdatesDriveDistanceMeters = new double[0];
    public double driveCurrent = 0;
    public double driveVoltage = 0;
}
