package frc.robot.subsystems.swerve.gyro;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class GyroInputs {

    public double gyroYawDegrees = 0;

    public double gyroPitchDegrees = 0;

    public double accelerationX = 0;

    public double accelerationY = 0;

    public double accelerationZ = 0;

    public double[] odometryUpdatesYawDegrees = new double[0];

    public double[] odometryUpdatesTimestamp = new double[0];

}
