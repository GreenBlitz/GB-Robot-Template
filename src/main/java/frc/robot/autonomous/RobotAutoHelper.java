package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.utils.auto.PathPlannerUtils;
import frc.utils.math.ToleranceMath;

public class RobotAutoHelper {

	public static Command followPath(PathPlannerPath path) {
		return AutoBuilder.followPath(path);
	}

	public static Command pathfindToPose(Pose2d targetPose, PathConstraints pathfindingConstraints) {
		return AutoBuilder.pathfindToPose(targetPose, pathfindingConstraints);
	}

	public static Command pathfindThenFollowPath(PathPlannerPath path, PathConstraints pathfindingConstraints) {
		return AutoBuilder.pathfindThenFollowPath(path, pathfindingConstraints);
	}

	public static Command followPathOrDriveToPathEnd(Robot robot, PathPlannerPath path) {
		return robot.getSwerve()
			.getCommandsBuilder()
			.followPathOrDriveToPathEnd(
				robot.getPoseEstimator()::getCurrentPose,
				path,
				() -> ToleranceMath.isNear(
					PathPlannerUtils.getAllianceRelativePose(PathPlannerUtils.getLastPathPose(path)),
					robot.getPoseEstimator().getCurrentPose(),
					AutonomousConstants.TARGET_ANGLE_TOLERANCE,
					AutonomousConstants.DISTANCE_FROM_TARGET_TOLERANCE_METERS
				)
			);
	}

}
