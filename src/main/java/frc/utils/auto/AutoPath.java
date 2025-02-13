package frc.utils.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.autonomous.AutonomousConstants;

import java.util.Optional;

public enum AutoPath {

	AUTO_LINE_1_TO_I(AutonomousConstants.LinkedWaypoints.AUTO_LINE1, AutonomousConstants.LinkedWaypoints.I),
	AUTO_LINE_1_TO_J(AutonomousConstants.LinkedWaypoints.AUTO_LINE1, AutonomousConstants.LinkedWaypoints.J),
	AUTO_LINE_1_TO_K(AutonomousConstants.LinkedWaypoints.AUTO_LINE1, AutonomousConstants.LinkedWaypoints.K),
	AUTO_LINE_2_TO_H(AutonomousConstants.LinkedWaypoints.AUTO_LINE2, AutonomousConstants.LinkedWaypoints.H),
	AUTO_LINE_2_TO_I(AutonomousConstants.LinkedWaypoints.AUTO_LINE2, AutonomousConstants.LinkedWaypoints.I),
	AUTO_LINE_2_TO_J(AutonomousConstants.LinkedWaypoints.AUTO_LINE2, AutonomousConstants.LinkedWaypoints.J),
	AUTO_LINE_2_TO_K(AutonomousConstants.LinkedWaypoints.AUTO_LINE2, AutonomousConstants.LinkedWaypoints.K),
	AUTO_LINE_3_TO_G(AutonomousConstants.LinkedWaypoints.AUTO_LINE3, AutonomousConstants.LinkedWaypoints.G),
	AUTO_LINE_3_TO_H(AutonomousConstants.LinkedWaypoints.AUTO_LINE3, AutonomousConstants.LinkedWaypoints.H),
	AUTO_LINE_3_TO_I(AutonomousConstants.LinkedWaypoints.AUTO_LINE3, AutonomousConstants.LinkedWaypoints.I),
	AUTO_LINE_3_TO_J(AutonomousConstants.LinkedWaypoints.AUTO_LINE3, AutonomousConstants.LinkedWaypoints.J),
	AUTO_LINE_4_TO_F(AutonomousConstants.LinkedWaypoints.AUTO_LINE3, AutonomousConstants.LinkedWaypoints.F),
	AUTO_LINE_4_TO_G(AutonomousConstants.LinkedWaypoints.AUTO_LINE3, AutonomousConstants.LinkedWaypoints.G),
	AUTO_LINE_4_TO_H(AutonomousConstants.LinkedWaypoints.AUTO_LINE3, AutonomousConstants.LinkedWaypoints.H),
	AUTO_LINE_4_TO_I(AutonomousConstants.LinkedWaypoints.AUTO_LINE3, AutonomousConstants.LinkedWaypoints.I),
	AUTO_LINE_5_TO_E(AutonomousConstants.LinkedWaypoints.AUTO_LINE3, AutonomousConstants.LinkedWaypoints.E),
	AUTO_LINE_5_TO_F(AutonomousConstants.LinkedWaypoints.AUTO_LINE3, AutonomousConstants.LinkedWaypoints.F),
	AUTO_LINE_5_TO_G(AutonomousConstants.LinkedWaypoints.AUTO_LINE3, AutonomousConstants.LinkedWaypoints.G),
	AUTO_LINE_5_TO_I(AutonomousConstants.LinkedWaypoints.AUTO_LINE3, AutonomousConstants.LinkedWaypoints.I),
	AUTO_LINE_6_TO_D(AutonomousConstants.LinkedWaypoints.AUTO_LINE3, AutonomousConstants.LinkedWaypoints.D),
	AUTO_LINE_6_TO_E(AutonomousConstants.LinkedWaypoints.AUTO_LINE3, AutonomousConstants.LinkedWaypoints.E),
	AUTO_LINE_6_TO_F(AutonomousConstants.LinkedWaypoints.AUTO_LINE3, AutonomousConstants.LinkedWaypoints.F),
	AUTO_LINE_7_TO_D(AutonomousConstants.LinkedWaypoints.AUTO_LINE3, AutonomousConstants.LinkedWaypoints.D),
	AUTO_LINE_7_TO_E(AutonomousConstants.LinkedWaypoints.AUTO_LINE3, AutonomousConstants.LinkedWaypoints.E),
	AUTO_LINE_7_TO_F(AutonomousConstants.LinkedWaypoints.AUTO_LINE3, AutonomousConstants.LinkedWaypoints.F),
	A_TO_UPPER_CORAL_STATION_2(AutonomousConstants.LinkedWaypoints.A, AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION2),
	A_TO_LOWER_CORAL_STATION_2(AutonomousConstants.LinkedWaypoints.A, AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION2),
	B_TO_LOWER_CORAL_STATION_2(AutonomousConstants.LinkedWaypoints.B, AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION2),
	B_TO_UPPER_CORAL_STATION_2(AutonomousConstants.LinkedWaypoints.B, AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION2),
	C_TO_LOWER_CORAL_STATION_2(AutonomousConstants.LinkedWaypoints.C, AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION2),
	D_TO_LOWER_CORAL_STATION_2(AutonomousConstants.LinkedWaypoints.D, AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION2),
	E_TO_LOWER_CORAL_STATION_2(AutonomousConstants.LinkedWaypoints.E, AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION2),
	F_TO_LOWER_CORAL_STATION_2(AutonomousConstants.LinkedWaypoints.F, AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION2),
	G_TO_LOWER_CORAL_STATION_2(AutonomousConstants.LinkedWaypoints.G, AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION2),
	H_TO_UPPER_CORAL_STATION_2(AutonomousConstants.LinkedWaypoints.H, AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION2),
	I_TO_UPPER_CORAL_STATION_2(AutonomousConstants.LinkedWaypoints.I, AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION2),
	J_TO_UPPER_CORAL_STATION_2(AutonomousConstants.LinkedWaypoints.J, AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION2),
	K_TO_UPPER_CORAL_STATION_2(AutonomousConstants.LinkedWaypoints.K, AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION2),
	L_TO_UPPER_CORAL_STATION_2(AutonomousConstants.LinkedWaypoints.L, AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION2),
	LOWER_CORAL_STATION_2_TO_A(AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION2, AutonomousConstants.LinkedWaypoints.A),
	LOWER_CORAL_STATION_2_TO_B(AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION2, AutonomousConstants.LinkedWaypoints.B),
	LOWER_CORAL_STATION_2_TO_C(AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION2, AutonomousConstants.LinkedWaypoints.C),
	LOWER_CORAL_STATION_2_TO_D(AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION2, AutonomousConstants.LinkedWaypoints.D),
	LOWER_CORAL_STATION_2_TO_E(AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION2, AutonomousConstants.LinkedWaypoints.E),
	LOWER_CORAL_STATION_2_TO_F(AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION2, AutonomousConstants.LinkedWaypoints.F),
	LOWER_CORAL_STATION_2_TO_G(AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION2, AutonomousConstants.LinkedWaypoints.G),
	UPPER_CORAL_STATION_2_TO_A(AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION2, AutonomousConstants.LinkedWaypoints.A),
	UPPER_CORAL_STATION_2_TO_B(AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION2, AutonomousConstants.LinkedWaypoints.B),
	UPPER_CORAL_STATION_2_TO_H(AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION2, AutonomousConstants.LinkedWaypoints.H),
	UPPER_CORAL_STATION_2_TO_I(AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION2, AutonomousConstants.LinkedWaypoints.I),
	UPPER_CORAL_STATION_2_TO_J(AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION2, AutonomousConstants.LinkedWaypoints.J),
	UPPER_CORAL_STATION_2_TO_K(AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION2, AutonomousConstants.LinkedWaypoints.K),
	UPPER_CORAL_STATION_2_TO_L(AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION2, AutonomousConstants.LinkedWaypoints.L);

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
		return PathPlannerUtil.getPathFromFile(getPathName());
	}

}
