package frc.utils.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.autonomous.AutonomousConstants;
import frc.utils.alerts.Alert;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Stream;

public class PathHelper {

	public static final Map<String, PathPlannerPath> PATH_PLANNER_PATHS = getAllPaths();

	private static List<String> getAllPathNames() {
		File[] pathFiles = new File(Filesystem.getDeployDirectory(), "pathplanner/paths").listFiles();

		if (pathFiles == null) {
			return new ArrayList<>();
		}

		return Stream.of(pathFiles)
			.filter(file -> !file.isDirectory())
			.map(File::getName)
			.filter(name -> name.endsWith(".path"))
			.map(name -> name.substring(0, name.lastIndexOf(".")))
			.sorted(String::compareToIgnoreCase)
			.toList();
	}

	private static Map<String, PathPlannerPath> getAllPaths() {
		Map<String, PathPlannerPath> paths = new HashMap<>();
		for (String pathName : getAllPathNames()) {
			try {
				paths.put(pathName, PathPlannerPath.fromPathFile(pathName));
			} catch (Exception exception) {
				new Alert(Alert.AlertType.ERROR, AutonomousConstants.LOG_PATH_PREFIX + "/" + exception.getMessage()).report();
			}
		}
		return paths;
	}

}
