package frc.robot.subsystems.swerve.module.extrainputs;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class DriveCouplingInputs {

	public Rotation2d uncoupledVelocityAnglesPerSecond = new Rotation2d();
	public Rotation2d[] uncoupledPositions = new Rotation2d[0];

}
