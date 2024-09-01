package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class GyroInputs {

	public boolean isConnected = false;
	public Rotation2d gyroYaw = new Rotation2d();
	public Rotation2d[] yawOdometrySamples = new Rotation2d[0];
	public double[] timestampOdometrySamples = new double[0];

}
