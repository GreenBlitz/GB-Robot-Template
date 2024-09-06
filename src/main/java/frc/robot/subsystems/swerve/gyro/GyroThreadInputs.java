package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class GyroThreadInputs {

	public Rotation2d[] yawOdometrySamples = new Rotation2d[0];
	public double[] timestampOdometrySamples = new double[0];

}
