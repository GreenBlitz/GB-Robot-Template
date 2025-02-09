package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.field.Field;
import frc.utils.auto.PathPlannerUtil;
import frc.utils.math.AngleTransform;

public class PathFollowingCommandsBuilder {

	public static Command followPath(PathPlannerPath path) {
		return AutoBuilder.followPath(path);
	}

	public static Command pathfindToPose(Pose2d targetPose, PathConstraints pathfindingConstraints) {
		return AutoBuilder.pathfindToPose(targetPose, pathfindingConstraints);
	}

	public static Command pathfindThenFollowPath(PathPlannerPath path, PathConstraints pathfindingConstraints) {
		return AutoBuilder.pathfindThenFollowPath(path, pathfindingConstraints);
	}

	public static Command pathfindThenFollowPath(
		PathPlannerPath path,
		PathConstraints pathfindingConstraints,
		double velocityBetweenPathfindingToPathFollowingMetersPerSecond
	) {
		return AutoBuilder
			.pathfindToPose(
				Field.getAllianceRelative(PathPlannerUtil.getPathStartingPose(path), true, true, AngleTransform.INVERT),
				pathfindingConstraints,
				velocityBetweenPathfindingToPathFollowingMetersPerSecond
			)
			.andThen(followPath(path));
	}

}
