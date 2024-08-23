package frc.robot.subsystems.swerve.modules.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class DriveThreadMetersInputs {

	@Deprecated
	public boolean isConnected = false;
	public double distanceMeters = 0;
	public double velocityMeters = 0;
	@Deprecated
	public Rotation2d angle = new Rotation2d();
	@Deprecated
	public Rotation2d velocity = new Rotation2d();
	@Deprecated
	public Rotation2d acceleration = new Rotation2d();
	@Deprecated
	public double current = 0;
	@Deprecated
	public double voltage = 0;
	public Rotation2d[] angleOdometrySamples = new Rotation2d[0];
	public double[] distanceMetersOdometrySamples = new double[0];

}
