package frc.utils.auto;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autonomous.AutonomousConstants;
import frc.utils.alerts.Alert;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

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

	public static Optional<RobotConfig> getGuiRobotConfig() {
		try {
			RobotConfig robotConfig = RobotConfig.fromGUISettings();
			return Optional.of(robotConfig);
		} catch (IOException ioException) {
			reportAlert(Alert.AlertType.ERROR, "GetGuiSettingsFileNotFoundAt");
		} catch (ParseException parseException) {
			reportAlert(Alert.AlertType.ERROR, "GuiSettingsParseFailedAt");
		}
		return Optional.empty();
	}

	public static void registerCommand(String commandName, Command command) {
		NamedCommands.registerCommand(commandName, command);
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

	private static void reportAlert(Alert.AlertType alertType, String message) {
		new Alert(alertType, AutonomousConstants.LOG_PATH_PREFIX + message).report();
	}

}
