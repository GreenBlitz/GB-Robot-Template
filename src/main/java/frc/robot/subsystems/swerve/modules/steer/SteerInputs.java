package frc.robot.subsystems.swerve.modules.steer;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utils.calibration.sysid.SysIdCalibrator;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class SteerInputs {

	public boolean isConnected = false;
	public Rotation2d angle = new Rotation2d();
	public Rotation2d velocity = new Rotation2d();
	public Rotation2d acceleration = new Rotation2d();
	public double voltage = 0;
	public Rotation2d[] angleOdometrySamples = new Rotation2d[0];

}
