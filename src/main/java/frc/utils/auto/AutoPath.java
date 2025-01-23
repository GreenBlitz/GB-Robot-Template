package frc.utils.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.autonomous.AutonomousConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public enum AutoPath {

	AUTO_LINE_2_TO_I(AutonomousConstants.LinkedWaypoints.AL2, AutonomousConstants.LinkedWaypoints.I),
	AUTO_LINE_4_TO_H(AutonomousConstants.LinkedWaypoints.AL4, AutonomousConstants.LinkedWaypoints.H),
	AUTO_LINE_6_TO_F(AutonomousConstants.LinkedWaypoints.AL6, AutonomousConstants.LinkedWaypoints.F),
	F_TO_LOWER_CORAL_STATION(AutonomousConstants.LinkedWaypoints.F, AutonomousConstants.LinkedWaypoints.LS),
	I_TO_UPPER_CORAL_STATION(AutonomousConstants.LinkedWaypoints.I, AutonomousConstants.LinkedWaypoints.US),
	LOWER_CORAL_STATION_TO_C(AutonomousConstants.LinkedWaypoints.LS, AutonomousConstants.LinkedWaypoints.C),
	UPPER_CORAL_STATION_TO_L(AutonomousConstants.LinkedWaypoints.US, AutonomousConstants.LinkedWaypoints.L);

	private final Pair<String, Pose2d> startingPoint;
	private final Pair<String, Pose2d> endPoint;

	AutoPath(Pair<String, Pose2d> startingPoint, Pair<String, Pose2d> endPoint) {
		this.startingPoint = startingPoint;
		this.endPoint = endPoint;
	}

	public Pair<String, Pose2d> getStartingPoint() {
		return startingPoint;
	}

	public Pair<String, Pose2d> getEndPoint() {
		return endPoint;
	}

	public String getPathName() {
		return startingPoint.getFirst() + "-" + endPoint.getFirst();
	}

	public Optional<PathPlannerPath> getPath() {
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
