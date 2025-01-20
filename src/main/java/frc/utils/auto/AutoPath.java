package frc.utils.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public enum AutoPath {

	AUTO_LINE_2_TO_I("AL2", new Translation2d(7.6, 6.17), "I", new Translation2d(5.25, 5.04)), // will use constants from Field or
																								// AutonomousConstants
	AUTO_LINE_4_TO_H("AL4", new Translation2d(7.6, 4.02), "H", new Translation2d(5.75, 4.19)),
	AUTO_LINE_6_TO_F("AL6", new Translation2d(7.6, 1.9), "F", new Translation2d(5.26, 3.02)),
	F_TO_LOWER_CORAL_STATION("F", new Translation2d(5.26, 3.02), "LS", new Translation2d(1.15, 0.97)),
	I_TO_UPPER_CORAL_STATION("I", new Translation2d(5.25, 5.04), "US", new Translation2d(1.15, 7.05)),
	LOWER_CORAL_STATION_TO_C("LS", new Translation2d(1.15, 0.97), "C", new Translation2d(3.72, 3.02)),
	UPPER_CORAL_STATION_TO_L("US", new Translation2d(1.15, 7.05), "L", new Translation2d(3.72, 5.04));

	private final Pair<String, Translation2d> startingPoint;
	private final Pair<String, Translation2d> endPoint;

	AutoPath(String startingPointName, Translation2d startingPointTranslation, String endPointName, Translation2d endPointTranslation) {
		this.startingPoint = Pair.of(startingPointName, startingPointTranslation);
		this.endPoint = Pair.of(endPointName, endPointTranslation);
	}

	public Pair<String, Translation2d> getStartingPoint() {
		return startingPoint;
	}

	public Pair<String, Translation2d> getEndPoint() {
		return endPoint;
	}

	public String getPathName() {
		return startingPoint.getFirst() + "-" + endPoint.getFirst();
	}

	public Optional<PathPlannerPath> getPathOptional() {
		return PathPlannerUtils.getPathFromFile(getPathName());
	}

	public static List<AutoPath> getAllAutoLinePaths() {
		ArrayList<AutoPath> autoLinePaths = new ArrayList<>();
		for (AutoPath autoPath : values()) {
			if (autoPath.getStartingPoint().getFirst().startsWith("AL")) {
				autoLinePaths.add(autoPath);
			}
		}
		return autoLinePaths;
	}

	public static List<AutoPath> getAllPathsToCoralStations() {
		ArrayList<AutoPath> pathsToCoralStations = new ArrayList<>();
		for (AutoPath autoPath : values()) {
			if (autoPath.getEndPoint().getFirst().equals("US") || autoPath.getEndPoint().getFirst().equals("LS")) {
				pathsToCoralStations.add(autoPath);
			}
		}
		return pathsToCoralStations;
	}

	public static List<AutoPath> getAllPathsFromCoralStations() {
		ArrayList<AutoPath> pathsFromCoralStations = new ArrayList<>();
		for (AutoPath autoPath : values()) {
			if (autoPath.getStartingPoint().getFirst().equals("US") || autoPath.getStartingPoint().getFirst().equals("LS")) {
				pathsFromCoralStations.add(autoPath);
			}
		}
		return pathsFromCoralStations;
	}

}
