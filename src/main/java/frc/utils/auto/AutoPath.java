package frc.utils.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.autonomous.AutonomousConstants;

import java.util.Optional;

public enum AutoPath {

	AUTO_LINE_2_TO_I(AutonomousConstants.LinkedWaypoints.AUTO_LINE2, AutonomousConstants.LinkedWaypoints.I),
	AUTO_LINE_4_TO_H(AutonomousConstants.LinkedWaypoints.AUTO_LINE4, AutonomousConstants.LinkedWaypoints.H),
	AUTO_LINE_6_TO_F(AutonomousConstants.LinkedWaypoints.AUTO_LINE6, AutonomousConstants.LinkedWaypoints.F),
	F_TO_LOWER_CORAL_STATION(AutonomousConstants.LinkedWaypoints.F, AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION),
	I_TO_UPPER_CORAL_STATION(AutonomousConstants.LinkedWaypoints.I, AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION),
	LOWER_CORAL_STATION_TO_C(AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION, AutonomousConstants.LinkedWaypoints.C),
	AUTO_LINE_1_TO_I(AutonomousConstants.LinkedWaypoints.AUTO_LINE1, AutonomousConstants.LinkedWaypoints.I),
	L_TO_UPPER_CORAL_STATION(AutonomousConstants.LinkedWaypoints.L, AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION),
	UPPER_CORAL_STATION_TO_K(AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION, AutonomousConstants.LinkedWaypoints.K),
	K_TO_UPPER_CORAL_STATION(AutonomousConstants.LinkedWaypoints.K, AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION),
	UPPER_CORAL_STATION_TO_J(AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION, AutonomousConstants.LinkedWaypoints.J),
	UPPER_CORAL_STATION_TO_L(AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION, AutonomousConstants.LinkedWaypoints.L);

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

}
