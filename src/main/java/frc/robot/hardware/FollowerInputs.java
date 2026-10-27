package frc.robot.hardware;

import org.wpilib.math.geometry.Angle2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class FollowerInputs {

	public FollowerData followerData = new FollowerData(true, new Angle2d(), 0);

	public record FollowerData(boolean connected, Angle2d position, double voltage) {}

}
