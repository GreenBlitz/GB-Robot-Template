package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.constants.field.Field;
import frc.robot.Robot;
import frc.utils.auto.PathPlannerUtils;
import frc.utils.math.ToleranceMath;

import java.util.function.Supplier;

public class SequencesBuilder {

	public static Command commandDuringPath(Robot robot, PathPlannerPath path, Supplier<Command> commandSupplier) {
		return new ParallelDeadlineGroup(commandSupplier.get(), followPathOrPathfindAndFollowPath(robot, path));
	}

	public static Command commandAfterPath(Robot robot, PathPlannerPath path, Supplier<Command> commandSupplier) {
		return new SequentialCommandGroup(followPathOrPathfindAndFollowPath(robot, path), commandSupplier.get());
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

	public static Command followPathOrPathfindAndFollowPath(Robot robot, PathPlannerPath path) {
		return robot.getSwerve()
			.getCommandsBuilder()
			.followPathOrPathfindAndFollowPath(robot.getPoseEstimator()::getCurrentPose, path)
			.until(
				() -> ToleranceMath.isNear(
					Field.getAllianceRelativePose(PathPlannerUtils.getPathStartingPose(path)),
					robot.getPoseEstimator().getCurrentPose(),
					AutonomousConstants.TARGET_ANGLE_TOLERANCE,
					AutonomousConstants.DISTANCE_FROM_TARGET_TOLERANCE_METERS
				)
			);
	}

}
