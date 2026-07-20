package frc.robot.hardware;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class FollowerInputs {

	public FollowerData followerData = new FollowerData(true, new Rotation2d(), 0);

	public record FollowerData(boolean connected, Rotation2d position, double voltage) {}

}
