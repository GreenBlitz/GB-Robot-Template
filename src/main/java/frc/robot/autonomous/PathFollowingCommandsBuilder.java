package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.constants.field.Field;
import frc.constants.field.enums.Branch;
import frc.robot.Robot;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.statemachine.StateMachineConstants;
import frc.utils.auto.PathPlannerUtil;
import frc.utils.math.AngleTransform;
import frc.utils.math.ToleranceMath;

import java.util.Optional;
import java.util.function.Supplier;

public class PathFollowingCommandsBuilder {

	public static Command commandDuringPath(
		Robot robot,
		PathPlannerPath path,
		Supplier<Command> commandSupplier,
		Optional<Branch> targetBranch,
		Pose2d tolerance
	) {
		return new ParallelCommandGroup(commandSupplier.get(), followAdjustedPath(robot, path, targetBranch, tolerance));
	}

	public static Command deadlinePathWithCommand(
		Robot robot,
		PathPlannerPath path,
		Supplier<Command> commandSupplier,
		Optional<Branch> targetBranch,
		Pose2d tolerance
	) {
		return new ParallelDeadlineGroup(commandSupplier.get(), followAdjustedPath(robot, path, targetBranch, tolerance));
	}

	public static Command commandAfterPath(
		Robot robot,
		PathPlannerPath path,
		Supplier<Command> commandSupplier,
		Optional<Branch> targetBranch,
		Pose2d tolerance
	) {
		return new SequentialCommandGroup(followAdjustedPath(robot, path, targetBranch, tolerance), commandSupplier.get());
	}

	public static Command scoreToBranch(Robot robot, PathPlannerPath path, Supplier<Command> commandSupplier, Optional<Branch> targetBranch) {
		return new ParallelDeadlineGroup(
			new SequentialCommandGroup(new WaitUntilCommand(() -> robot.getRobotCommander().isReadyToScore()), commandSupplier.get()),
			followAdjustedPathWithoutStop(robot, path, targetBranch)
		);
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
				Field.getAllianceRelative(PathPlannerUtil.getPathStartingPose(path), true, true, AngleTransform.INVERT),
				pathfindingConstraints,
				velocityBetweenPathfindingToPathFollowingMetersPerSecond
			)
			.andThen(followPath(path));
	}

	public static Command followPathOrPathfindAndFollowPath(Robot robot, PathPlannerPath path) {
		return new ConditionalCommand(
			followPath(path),
			pathfindThenFollowPath(path, AutonomousConstants.getRealTimeConstraints(robot.getSwerve())),
			() -> PathPlannerUtil.isRobotInPathfindingDeadband(
				robot.getPoseEstimator().getEstimatedPose(),
				Field.getAllianceRelative(PathPlannerUtil.getPathStartingPose(path), true, true, AngleTransform.INVERT)
			)
		);
	}

	public static Command moveToPoseByPID(Robot robot, Pose2d targetPose) {
		return robot.getSwerve().getCommandsBuilder().moveToPoseByPID(robot.getPoseEstimator()::getEstimatedPose, targetPose);
	}

	public static Command followAdjustedPath(Robot robot, PathPlannerPath path, Optional<Branch> targetBranch, Pose2d tolerance) {
		return robot.getSwerve()
			.asSubsystemCommand(
				followPathOrPathfindAndFollowPath(robot, path)
					.andThen(
						moveToPoseByPID(
							robot,
							targetBranch
								.map(
									branch -> ScoringHelpers
										.getRobotBranchScoringPose(branch, StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS)
								)
								.orElse(Field.getAllianceRelative(PathPlannerUtil.getLastPathPose(path), true, true, AngleTransform.INVERT))
						)
					)
					.until(
						() -> targetBranch.map(branch -> robot.getRobotCommander().isAtBranchScoringPose(branch))
							.orElse(
								ToleranceMath.isNear(
									Field.getAllianceRelative(PathPlannerUtil.getLastPathPose(path), true, true, AngleTransform.INVERT),
									robot.getPoseEstimator().getEstimatedPose(),
									tolerance
								)
							)
					)
					.andThen(robot.getSwerve().getCommandsBuilder().resetTargetSpeeds()),
				"Follow Adjusted " + path.name
			);
	}

	public static Command followAdjustedPathWithoutStop(Robot robot, PathPlannerPath path, Optional<Branch> targetBranch) {
		return robot.getSwerve()
			.asSubsystemCommand(
				followPathOrPathfindAndFollowPath(robot, path).andThen(
					moveToPoseByPID(
						robot,
						targetBranch
							.map(
								branch -> ScoringHelpers
									.getRobotBranchScoringPose(branch, StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS)
							)
							.orElse(Field.getAllianceRelative(PathPlannerUtil.getLastPathPose(path), true, true, AngleTransform.INVERT))
					)
				),
				"Follow Adjusted " + path + " without stop"
			);
	}

}
