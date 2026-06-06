package frc.utils.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import org.wpilib.math.util.Pair;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.util.struct.parser.ParseException;
import frc.robot.autonomous.AutonomousConstants;
import frc.robot.autonomous.PathFollowingCommandsBuilder;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.alerts.Alert;
import frc.utils.math.ToleranceMath;
import org.littletonrobotics.junction.Logger;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class PathPlannerUtil {

	private static List<Pair<Translation2d, Translation2d>> dynamicObstacles = List.of();

	public static void configPathPlanner(
		Supplier<Pose2d> poseSupplier,
		Consumer<Pose2d> resetPose,
		Supplier<ChassisVelocities> robotRelativeVelocitiesSupplier,
		Consumer<ChassisVelocities> robotRelativeVelocitiesSetter,
		PPHolonomicDriveController holonomicDriveController,
		RobotConfig robotConfig,
		BooleanSupplier shouldFlipPath,
		GBSubsystem... driveRequirements
	) {
		AutoBuilder.configure(
			poseSupplier,
			resetPose,
			robotRelativeVelocitiesSupplier,
			robotRelativeVelocitiesSetter,
			holonomicDriveController,
			robotConfig,
			shouldFlipPath,
			driveRequirements
		);
	}

	public static void setupPathPlannerLogging() {
		PathPlannerLogging.setLogActivePathCallback(
			activePath -> Logger.recordOutput(AutonomousConstants.LOG_PATH_PREFIX + "/ActivePath", activePath.toArray(Pose2d[]::new))
		);
		PathPlannerLogging
			.setLogTargetPoseCallback(targetPose -> Logger.recordOutput(AutonomousConstants.LOG_PATH_PREFIX + "/TargetPose", targetPose));
	}

	public static void startPathfinder() {
		setPathfinder(new LocalADStar());
		scheduleWarmup();
	}

	public static void setPathfinder(Pathfinder pathfinder) {
		Pathfinding.setPathfinder(pathfinder);
	}

	public static void scheduleWarmup() {
		CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
	}

	public static Optional<RobotConfig> getGuiRobotConfig() {
		try {
			RobotConfig robotConfig = RobotConfig.fromGUISettings();
			return Optional.of(robotConfig);
		} catch (IOException ioException) {
			reportAlert(Alert.AlertType.ERROR, "GuiSettingsFileNotFoundAt");
		} catch (ParseException parseException) {
			reportAlert(Alert.AlertType.ERROR, "GuiSettingsParseFailedAt");
		}
		return Optional.empty();
	}

	public static void registerCommand(String commandName, Command command) {
		NamedCommands.registerCommand(commandName, command);
	}

	public static Pose2d getPathStartingPose(PathPlannerPath path) {
		return new Pose2d(path.getPathPoses().get(0).getTranslation(), path.getIdealStartingState().rotation());
	}

	public static Pose2d getLastPathPose(PathPlannerPath path) {
		return new Pose2d(path.getPathPoses().get(path.getPathPoses().size() - 1).getTranslation(), path.getGoalEndState().rotation());
	}

	public static Command createPathDuringRuntime(Pose2d currentPose, Pose2d targetPose, PathConstraints constraints) {
		List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(currentPose, targetPose);
		PathPlannerPath path = new PathPlannerPath(bezierPoints, constraints, null, new GoalEndState(0, targetPose.getRotation()));
		path.preventFlipping = true;
		return PathFollowingCommandsBuilder.followPath(path);
	}

	public static void setDynamicObstacles(List<Pair<Translation2d, Translation2d>> obstacles, Pose2d currentPose) {
		dynamicObstacles = obstacles;
		Pathfinding.setDynamicObstacles(obstacles, currentPose.getTranslation());
	}

	public static void addDynamicObstacles(List<Pair<Translation2d, Translation2d>> obstacles, Pose2d currentPose) {
		List<Pair<Translation2d, Translation2d>> allObstacles = new ArrayList<>();
		allObstacles.addAll(dynamicObstacles);
		allObstacles.addAll(obstacles);
		setDynamicObstacles(allObstacles, currentPose);
	}

	public static void removeAllDynamicObstacles(Pose2d currentPose) {
		setDynamicObstacles(List.of(), currentPose);
	}

	public static boolean isRobotInPathfindingDeadband(Pose2d currentPose, Pose2d targetPose) {
		return ToleranceMath.isNear(targetPose.getTranslation(), currentPose.getTranslation(), AutonomousConstants.PATHFINDING_DEADBAND_METERS);
	}

	private static void reportAlert(Alert.AlertType alertType, String message) {
		new Alert(alertType, AutonomousConstants.LOG_PATH_PREFIX + "/" + message).report();
	}

}
