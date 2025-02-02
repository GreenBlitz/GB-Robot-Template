package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.constants.field.Field;
import frc.robot.Robot;
import frc.utils.auto.PathPlannerUtil;

import java.util.function.Supplier;

public class SequencesBuilder {

	public static Command commandDuringPath(Robot robot, PathPlannerPath path, Supplier<Command> commandSupplier) {
		return new ParallelCommandGroup(commandSupplier.get(), followAdjustedPath(robot, path));
	}

	public static Command commandAfterPath(Robot robot, PathPlannerPath path, Supplier<Command> commandSupplier) {
		return new SequentialCommandGroup(followAdjustedPath(robot, path), commandSupplier.get());
	}

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
						Field.getAllianceRelativePose(PathPlannerUtil.getPathStartingPose(path)),
						pathfindingConstraints,
						velocityBetweenPathfindingToPathFollowingMetersPerSecond
				)
				.andThen(followPath(path));
	}

	public static Command followPathOrPathfindAndFollowPath(Robot robot, PathPlannerPath path) {
		return new ConditionalCommand(
			followPath(path),
			pathfindThenFollowPath(path, AutonomousConstants.REAL_TIME_CONSTRAINTS),
			() -> PathPlannerUtil.isRobotInPathfindingDeadband(
				robot.getPoseEstimator().getCurrentPose(),
				Field.getAllianceRelativePose(PathPlannerUtil.getPathStartingPose(path))
			)
		);
	}

	public static Command pidToPose(Robot robot, Pose2d targetPose) {
		return robot.getSwerve()
			.getCommandsBuilder()
			.pidToPose(robot.getPoseEstimator()::getCurrentPose, targetPose)
			.until(() -> PathPlannerUtil.isRobotInAutonomousTolerances(robot.getPoseEstimator().getCurrentPose(), targetPose));
	}

	public static Command followAdjustedPath(Robot robot, PathPlannerPath path) {
		return followPathOrPathfindAndFollowPath(robot, path)
			.andThen(pidToPose(robot, Field.getAllianceRelativePose(PathPlannerUtil.getLastPathPose(path))));
	}

}
