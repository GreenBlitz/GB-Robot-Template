package frc.robot.subsystems.swerve.modules.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class DriveThreadMetersInputs {

	public double distanceMeters = 0;
	public double velocityMeters = 0;
	public Rotation2d[] angleOdometrySamples = new Rotation2d[0];
	public double[] distanceMetersOdometrySamples = new double[0];

}
