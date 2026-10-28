package frc.robot.subsystems.swerve.module.extrainputs;

import frc.utils.costomUnits.Angle2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class DriveInputs {

	public Angle2d uncoupledVelocityAnglesPerSecond = new Angle2d();
	public Angle2d[] uncoupledPositions = new Angle2d[0];

	public double velocityMetersPerSecond = 0;
	public double[] positionsMeters = new double[0];

}
