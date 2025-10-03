package frc.robot.hardware;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class FollowerInputs {

	public boolean connected = true;
	public Rotation2d position = new Rotation2d();
	public double voltage = 0;

}
