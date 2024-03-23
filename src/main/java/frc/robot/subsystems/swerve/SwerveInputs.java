package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class SwerveInputs {

    public double[] odometryUpdatesYawDegrees = new double[0];
    public double gyroYawDegrees = 0;
    public double gyroPitchDegrees = 0;
    public double accelerationX = 0;
    public double accelerationY = 0;
    public double accelerationZ = 0;

    public double[] odometryUpdatesTimestamp = new double[0];


}
