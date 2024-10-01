package frc.robot.subsystems.swerve.module.extrainputs;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class DriveInputs {

	public double velocityMetersPerSecond = 0;
	public Rotation2d uncoupledVelocityPerSecond = new Rotation2d();
	public double[] positionsMeters = new double[0];
	public Rotation2d[] uncoupledPositions = new Rotation2d[0];

}
