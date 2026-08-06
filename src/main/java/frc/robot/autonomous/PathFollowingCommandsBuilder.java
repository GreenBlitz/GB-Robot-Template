package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.constants.field.Field;
import frc.robot.subsystems.swerve.Swerve;
import frc.utils.auto.PathPlannerUtil;
import frc.utils.math.ToleranceMath;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class PathFollowingCommandsBuilder {

	public static Command commandDuringPath(
		Swerve swerve,
		Supplier<Pose2d> currentPose,
		PathPlannerPath path,
		PathConstraints pathfindingConstraints,
		Supplier<Command> commandSupplier,
		Pose2d regularIsNearEndOfPathTolerance,
		Pose2d stuckIsNearEndOfPathTolerance,
		double stuckDebounceSeconds,
		String logPath
	) {
		return new ParallelCommandGroup(
			commandSupplier.get(),
			followAdjustedPathThenStop(
				swerve,
				currentPose,
				path,
				pathfindingConstraints,
				regularIsNearEndOfPathTolerance,
				stuckIsNearEndOfPathTolerance,
				stuckDebounceSeconds
			)
		);
	}

	public static Command deadlinePathWithCommand(
		Swerve swerve,
		Supplier<Pose2d> currentPose,
		PathPlannerPath path,
		PathConstraints pathfindingConstraints,
		Supplier<Command> commandSupplier
	) {
		return new ParallelDeadlineGroup(commandSupplier.get(), followAdjustedPath(swerve, currentPose, path, pathfindingConstraints));
	}

	public static Command commandAfterPath(
		Swerve swerve,
		Supplier<Pose2d> currentPose,
		PathPlannerPath path,
		PathConstraints pathfindingConstraints,
		Supplier<Command> commandSupplier,
		Pose2d regularIsNearEndOfPathTolerance,
		Pose2d stuckIsNearEndOfPathTolerance,
		double stuckDebounceSeconds
	) {
		return new SequentialCommandGroup(
			followAdjustedPathThenStop(
				swerve,
				currentPose,
				path,
				pathfindingConstraints,
				regularIsNearEndOfPathTolerance,
				stuckIsNearEndOfPathTolerance,
				stuckDebounceSeconds
			),
			commandSupplier.get()
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
				Field.getAllianceRelative(PathPlannerUtil.getPathStartingPose(path)),
				pathfindingConstraints,
				velocityBetweenPathfindingToPathFollowingMetersPerSecond
			)
			.andThen(followPath(path));
	}

	public static Command followPathOrPathfindAndFollowPath(
		PathPlannerPath path,
		Supplier<Pose2d> currentPose,
		PathConstraints pathfindingConstraints
	) {
		return new ConditionalCommand(
			followPath(path),
			pathfindThenFollowPath(path, pathfindingConstraints),
			() -> PathPlannerUtil
				.isRobotInPathfindingDeadband(currentPose.get(), Field.getAllianceRelative(PathPlannerUtil.getPathStartingPose(path)))
		);
	}

	public static Command followAdjustedPath(
		Swerve swerve,
		Supplier<Pose2d> currentPose,
		PathPlannerPath path,
		PathConstraints pathfindingConstraints
	) {
		return swerve.asSubsystemCommand(
			followPathOrPathfindAndFollowPath(path, currentPose, pathfindingConstraints).andThen(
				swerve.getCommandsBuilder().moveToPoseByPID(currentPose, Field.getAllianceRelative(PathPlannerUtil.getLastPathPose(path)))
			),
			"Follow Adjusted " + path.name
		);
	}

	public static Command followAdjustedPathThenStop(
		Swerve swerve,
		Supplier<Pose2d> currentPose,
		PathPlannerPath path,
		PathConstraints pathfindingConstraints,
		Pose2d regularIsNearEndOfPathTolerance,
		Pose2d stuckIsNearEndOfPathTolerance,
		double stuckDebounceSeconds
	) {
		return followAdjustedPath(swerve, currentPose, path, pathfindingConstraints)
			.until(isNearEndOfPath(path, currentPose, regularIsNearEndOfPathTolerance, stuckIsNearEndOfPathTolerance, stuckDebounceSeconds))
			.andThen(swerve.getCommandsBuilder().resetTargetSpeeds());
	}

	private static BooleanSupplier isNearEndOfPath(
		PathPlannerPath path,
		Supplier<Pose2d> currentPose,
		Pose2d regularTolerance,
		Pose2d stuckTolerance,
		double stuckDebounceSeconds
	) {
		Debouncer stuckDebouncer = new Debouncer(stuckDebounceSeconds, Debouncer.DebounceType.kRising);

		return () -> {
			Pose2d targetPose = Field.getAllianceRelative(PathPlannerUtil.getLastPathPose(path));
			Pose2d current = currentPose.get();

			boolean isNearRegularTolerance = ToleranceMath.isNear(targetPose, current, regularTolerance);
			boolean isNearStuckTolerance = ToleranceMath.isNear(targetPose, current, stuckTolerance);

			if (isNearRegularTolerance) {
				return true;
			}

			return stuckDebouncer.calculate(isNearStuckTolerance);
		};
	}

}
