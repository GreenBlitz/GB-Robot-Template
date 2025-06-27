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
import frc.robot.subsystems.swerve.Swerve;
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

	public static Command scoreToNet(Robot robot, PathPlannerPath path, Supplier<Command> netCommandSupplier, Optional<Branch> targetBranch) {
		Command netAutoReleaseAfterDelay = new SequentialCommandGroup(new WaitCommand(2), netCommandSupplier.get());
		return new ParallelDeadlineGroup(
			new SequentialCommandGroup(new WaitUntilCommand(() -> robot.getRobotCommander().isReadyForNetForAuto()), netCommandSupplier.get()),
			followAdjustedPath(robot, path, targetBranch, AutonomousConstants.TARGET_POSE_TOLERANCES)
//					.andThen(netAutoReleaseAfterDelay.onlyIf(
//					() -> SwerveMath.isStill(robot.getSwerve().getRobotRelativeVelocity(), AutonomousConstants.NET_AUTO_RELEASE_DEADBANDS)
//						&& !robot.getRobotCommander().isReadyForNetForAuto()
//				)
//			)
		);
	}


	public static Command followPath(PathPlannerPath path) {
		return AutoBuilder.followPath(path);
	}

	public static Command pathfindToPose(Pose2d targetPose, PathConstraints pathfindingConstraints) {
		return AutoBuilder.pathfindToPose(targetPose, pathfindingConstraints);
	}

	public static Command pathfindToPose(Pose2d targetPose, PathConstraints pathfindingConstraints, double goalEndVelocityMetersPerSecond) {
		return AutoBuilder.pathfindToPose(targetPose, pathfindingConstraints, goalEndVelocityMetersPerSecond);
	}

	public static Command pathfindThenFollowPath(PathPlannerPath path, PathConstraints pathfindingConstraints) {
		return AutoBuilder.pathfindThenFollowPath(path, pathfindingConstraints);
	}

	public static Command pathfindThenFollowPath(
		PathPlannerPath path,
		PathConstraints pathfindingConstraints,
		double velocityBetweenPathfindingToPathFollowingMetersPerSecond
	) {
		return pathfindToPose(
			Field.getAllianceRelative(PathPlannerUtil.getPathStartingPose(path), true, true, AngleTransform.INVERT),
			pathfindingConstraints,
			velocityBetweenPathfindingToPathFollowingMetersPerSecond
		).andThen(followPath(path));
	}

	public static Command followPathOrPathfindAndFollowPath(Swerve swerve, PathPlannerPath path, Supplier<Pose2d> currentPose) {
		return new ConditionalCommand(
			followPath(path),
			pathfindThenFollowPath(path, AutonomousConstants.getRealTimeConstraints(swerve)),
			() -> PathPlannerUtil.isRobotInPathfindingDeadband(
				currentPose.get(),
				Field.getAllianceRelative(PathPlannerUtil.getPathStartingPose(path), true, true, AngleTransform.INVERT)
			)
		);
	}

	public static Command followAdjustedPath(Robot robot, PathPlannerPath path, Optional<Branch> targetBranch, Pose2d tolerance) {
		return robot.getSwerve()
			.asSubsystemCommand(
				followPathOrPathfindAndFollowPath(robot.getSwerve(), path, () -> robot.getPoseEstimator().getEstimatedPose())
					.andThen(
						robot.getSwerve()
							.getCommandsBuilder()
							.moveToPoseByPID(
								robot.getPoseEstimator()::getEstimatedPose,
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
				followPathOrPathfindAndFollowPath(robot.getSwerve(), path, () -> robot.getPoseEstimator().getEstimatedPose()).andThen(
					robot.getSwerve()
						.getCommandsBuilder()
						.moveToPoseByPID(
							robot.getPoseEstimator()::getEstimatedPose,
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
