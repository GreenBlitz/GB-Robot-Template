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
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autonomous.AutonomousConstants;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.alerts.Alert;
import org.json.simple.parser.ParseException;
import frc.utils.math.ToleranceMath;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class PathPlannerUtils {

	private static List<Pair<Translation2d, Translation2d>> dynamicObstacles = List.of();

	public static void startPathfinder() {
		setPathfinder(new LocalADStar());
		scheduleWarmup();
	}

	public static void setPathfinder(Pathfinder pathfinder) {
		Pathfinding.setPathfinder(pathfinder);
	}

	public static void scheduleWarmup() {
		PathfindingCommand.warmupCommand().schedule();
	}


	private static void reportAlert(Alert.AlertType alertType, String message) {
		new Alert(alertType, AutonomousConstants.LOG_PATH_PREFIX + message).report();
	}

	public static Optional<RobotConfig> getGuiRobotConfig() {
		try {
			RobotConfig robotConfig = RobotConfig.fromGUISettings();
			return Optional.of(robotConfig);
		} catch (IOException ioException) {
			reportAlert(Alert.AlertType.ERROR, "GetGuiSettingsFailNotFoundAt");
		} catch (ParseException parseException) {
			reportAlert(Alert.AlertType.ERROR, "GuiSettingsParseFailedAt");
		}
		return Optional.empty();
	}

	public static void configPathPlanner(
		Supplier<Pose2d> poseSupplier,
		Consumer<Pose2d> resetPose,
		Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
		Consumer<ChassisSpeeds> robotRelativeSpeedsSetter,
		PPHolonomicDriveController holonomicDriveController,
		RobotConfig robotConfig,
		BooleanSupplier shouldFlipPath,
		GBSubsystem... driveRequirements
	) {
		AutoBuilder.configure(
			poseSupplier,
			resetPose,
			robotRelativeSpeedsSupplier,
			robotRelativeSpeedsSetter,
			holonomicDriveController,
			robotConfig,
			shouldFlipPath,
			driveRequirements
		);
	}

	public static void registerCommand(String commandName, Command command) {
		NamedCommands.registerCommand(commandName, command);
	}

	public static Pose2d getAllianceRelativePose(Pose2d bluePose) {
		if (AutoBuilder.shouldFlip()) {
			return FlippingUtil.flipFieldPose(bluePose);
		}
		return bluePose;
	}

	static Optional<PathPlannerPath> getPathFromFile(String pathName) {
		try {
			return Optional.of(PathPlannerPath.fromPathFile(pathName));
		} catch (Exception exception) {
			reportAlert(Alert.AlertType.ERROR, exception.getMessage());
		}
		return Optional.empty();
	}

	public static Pose2d getPathStartingPose(PathPlannerPath path) {
		return new Pose2d(path.getPathPoses().get(0).getTranslation(), path.getIdealStartingState().rotation());
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

	public static boolean isRobotCloseToPathBeginning(PathPlannerPath path, Supplier<Pose2d> currentPose, double toleranceMeters) {
		return ToleranceMath
			.isNear(getAllianceRelativePose(path.getPathPoses().get(0)).getTranslation(), currentPose.get().getTranslation(), toleranceMeters);
	}

	public static Pose2d getLastPathPose(PathPlannerPath path) {
		return new Pose2d(path.getPathPoses().get(path.getPathPoses().size() - 1).getTranslation(), path.getGoalEndState().rotation());
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

	public static Command createPathOnTheFly(Pose2d currentPose, Pose2d targetPose, PathConstraints constraints) {
		List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(currentPose, targetPose);
		PathPlannerPath path = new PathPlannerPath(bezierPoints, constraints, null, new GoalEndState(0, targetPose.getRotation()));
		path.preventFlipping = true;
		return followPath(path);
	}

	@SafeVarargs
	public static String getAutoName(String autoName, Optional<PathPlannerPath>... paths) {
		for (Optional<PathPlannerPath> path : paths) {
			if (path.isEmpty()) {
				return autoName + " (partial)";
			}
		}
		return autoName;
	}

}
