package frc.utils.auto;

import java.util.ArrayList;
import java.util.List;

public class PathHelper {


	public static List<AutoPath> getAllStartingAndScoringFirstObjectPaths() {
		ArrayList<AutoPath> autoLinePaths = new ArrayList<>();
		for (AutoPath autoPath : AutoPath.values()) {
			if (autoPath.getStartingPoint().getFirst().startsWith("AL")) {
				autoLinePaths.add(autoPath);
			}
		}
		return autoLinePaths;
	}

	public static List<AutoPath> getAllIntakingPaths() {
		ArrayList<AutoPath> pathsToCoralStations = new ArrayList<>();
		for (AutoPath autoPath : AutoPath.values()) {
			if (autoPath.getEndPoint().getFirst().startsWith("US") || autoPath.getEndPoint().getFirst().startsWith("LS")) {
				pathsToCoralStations.add(autoPath);
			}
		}
		return pathsToCoralStations;
	}

	public static List<AutoPath> getAllScoringPathsFromCoralStations() {
		ArrayList<AutoPath> pathsFromCoralStations = new ArrayList<>();
		for (AutoPath autoPath : AutoPath.values()) {
			if (autoPath.getStartingPoint().getFirst().startsWith("US") || autoPath.getStartingPoint().getFirst().startsWith("LS")) {
				pathsFromCoralStations.add(autoPath);
			}
		}
		return pathsFromCoralStations;
	}

}
