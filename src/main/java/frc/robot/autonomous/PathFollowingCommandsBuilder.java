package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.constants.field.Field;
import frc.robot.subsystems.swerve.Swerve;
import frc.utils.auto.PathPlannerUtil;
import frc.utils.math.ToleranceMath;
import org.littletonrobotics.junction.Logger;

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
		double stuckDebounceTimeSeconds,
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
				stuckDebounceTimeSeconds,
				logPath
			)
		);
	}

	public static Command deadlinePathWithCommand(
		Swerve swerve,
		Supplier<Pose2d> currentPose,
		PathPlannerPath path,
		PathConstraints pathfindingConstraints,
		Supplier<Command> commandSupplier,
		String logPath
	) {
		return new ParallelDeadlineGroup(commandSupplier.get(), followAdjustedPath(swerve, currentPose, path, pathfindingConstraints, logPath));
	}

	public static Command commandAfterPath(
		Swerve swerve,
		Supplier<Pose2d> currentPose,
		PathPlannerPath path,
		PathConstraints pathfindingConstraints,
		Supplier<Command> commandSupplier,
		Pose2d regularIsNearEndOfPathTolerance,
		Pose2d stuckIsNearEndOfPathTolerance,
		double stuckDebounceTimeSeconds,
		String logPath
	) {
		return new SequentialCommandGroup(
			followAdjustedPathThenStop(
				swerve,
				currentPose,
				path,
				pathfindingConstraints,
				regularIsNearEndOfPathTolerance,
				stuckIsNearEndOfPathTolerance,
				stuckDebounceTimeSeconds,
				logPath
			),
			commandSupplier.get()
		);
	}


	public static Command followPath(PathPlannerPath path, String logPath) {
		return AutoBuilder.followPath(path)
			.alongWith(new InstantCommand(() -> Logger.recordOutput(logPath + "/CurrentCommand", "followPath : " + path.name)));
	}

	public static Command pathfindToPose(Pose2d targetPose, PathConstraints pathfindingConstraints, String logPath) {
		return AutoBuilder.pathfindToPose(targetPose, pathfindingConstraints)
			.alongWith(new InstantCommand(() -> Logger.recordOutput(logPath + "/CurrentCommand", "pathfindToPose:" + targetPose)));
	}

	public static Command pathfindThenFollowPath(PathPlannerPath path, PathConstraints pathfindingConstraints, String logPath) {
		return AutoBuilder.pathfindThenFollowPath(path, pathfindingConstraints)
			.alongWith(new InstantCommand(() -> Logger.recordOutput(logPath + "/CurrentCommand", "pathfindThenFollowPath: " + path.name)));
	}

	public static Command pathfindThenFollowPath(
		PathPlannerPath path,
		PathConstraints pathfindingConstraints,
		double velocityBetweenPathfindingToPathFollowingMetersPerSecond,
		String logPath
	) {
		return AutoBuilder
			.pathfindToPose(
				Field.getAllianceRelative(PathPlannerUtil.getPathStartingPose(path)),
				pathfindingConstraints,
				velocityBetweenPathfindingToPathFollowingMetersPerSecond
			)
			.andThen(followPath(path, logPath));
	}

	public static Command followPathOrPathfindAndFollowPath(
		PathPlannerPath path,
		Supplier<Pose2d> currentPose,
		PathConstraints pathfindingConstraints,
		String logPath
	) {
		return new ConditionalCommand(
			followPath(path, logPath),
			pathfindThenFollowPath(path, pathfindingConstraints, logPath),
			() -> PathPlannerUtil
				.isRobotInPathfindingDeadband(currentPose.get(), Field.getAllianceRelative(PathPlannerUtil.getPathStartingPose(path)))
		);
	}

	public static Command followAdjustedPath(
		Swerve swerve,
		Supplier<Pose2d> currentPose,
		PathPlannerPath path,
		PathConstraints pathfindingConstraints,
		String logPath
	) {
		return swerve.asSubsystemCommand(
			followPathOrPathfindAndFollowPath(path, currentPose, pathfindingConstraints, logPath).andThen(
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
		double stuckDebounceTimeSeconds,
		String logPath
	) {
		return followAdjustedPath(swerve, currentPose, path, pathfindingConstraints, logPath)
			.until(isNearEndOfPath(path, currentPose, regularIsNearEndOfPathTolerance, stuckIsNearEndOfPathTolerance, stuckDebounceTimeSeconds))
			.andThen(swerve.getCommandsBuilder().resetTargetSpeeds());
	}

	private static BooleanSupplier isNearEndOfPath(
		PathPlannerPath path,
		Supplier<Pose2d> currentPose,
		Pose2d regularTolerance,
		Pose2d stuckTolerance,
		double stuckDebounceTimeSeconds
	) {
		Debouncer stuckDebouncer = new Debouncer(stuckDebounceTimeSeconds, Debouncer.DebounceType.kRising);

		return () -> {
			Pose2d targetPose = Field.getAllianceRelative(PathPlannerUtil.getLastPathPose(path));
			Pose2d current = currentPose.get();

			boolean isNearRegularTolerance = ToleranceMath.isNear(targetPose, current, regularTolerance);
			boolean isNearStuckTolerance = ToleranceMath.isNear(targetPose, current, stuckTolerance);
			return isNearRegularTolerance || stuckDebouncer.calculate(isNearStuckTolerance);
		};
	}

}
