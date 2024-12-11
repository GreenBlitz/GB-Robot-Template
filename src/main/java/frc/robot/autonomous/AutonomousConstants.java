package frc.robot.autonomous;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.utils.auto.PathPlannerUtils;

import java.util.function.BiFunction;
import java.util.function.Supplier;

public class AutonomousConstants {

	public static double PATHFIND_OR_FOLLOW_PATH_TOLERANCE_METERS = 0.5;

	public static BiFunction<
		Robot,
		Supplier<Pose2d>,
		Command> DRIVE_TO_POSE_FUNCTION_GETTER = (robot, pose2dSupplier) -> robot.getSwerve()
			.getCommandsBuilder()
			.driveToPose(robot.getPoseEstimator()::getCurrentPose, pose2dSupplier);


	public static BiFunction<
		Robot,
		String,
		Command> PATHFIND_OR_FOLLOW_PATH_FUNCTION = (robot, pathName) -> PathPlannerUtils.pathfindOrFollowPath(
			pathName,
			(pose2dSupplier) -> AutonomousConstants.DRIVE_TO_POSE_FUNCTION_GETTER.apply(robot, pose2dSupplier),
			AutonomousConstants.PATHFIND_OR_FOLLOW_PATH_TOLERANCE_METERS
		);

	public static Command INTAKE_COMMAND = NamedCommands.getCommand("INTAKE");

	public static Command BEFORE_SHOOTING_COMMAND = NamedCommands.getCommand("PRE_SPEAKER");

	public static Command SHOOTING_COMMAND = NamedCommands.getCommand("SPEAKER");

}
