package frc.robot.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class RobotAutoHelper {

	public static Command followPathOrDriveToPathEnd(Robot robot, PathPlannerPath path) {
		return robot.getSwerve().getCommandsBuilder().followPathOrDriveToPathEnd(robot.getPoseEstimator()::getCurrentPose, path);
	}

}
