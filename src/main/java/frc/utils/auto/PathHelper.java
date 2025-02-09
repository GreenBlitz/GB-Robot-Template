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
            if (autoPath.getEndPoint().getFirst().equals("US") || autoPath.getEndPoint().getFirst().equals("LS")) {
                pathsToCoralStations.add(autoPath);
            }
        }
        return pathsToCoralStations;
    }

    public static List<AutoPath> getAllScoringPathsFromCoralStations() {
        ArrayList<AutoPath> pathsFromCoralStations = new ArrayList<>();
        for (AutoPath autoPath : AutoPath.values()) {
            if (autoPath.getStartingPoint().getFirst().equals("US") || autoPath.getStartingPoint().getFirst().equals("LS")) {
                pathsFromCoralStations.add(autoPath);
            }
        }
        return pathsFromCoralStations;
    }

}
