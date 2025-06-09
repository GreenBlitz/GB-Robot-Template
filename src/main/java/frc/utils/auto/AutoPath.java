package frc.utils.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import frc.constants.field.enums.Branch;
import frc.robot.autonomous.AutonomousConstants;

import java.util.Optional;

public enum AutoPath {

	AUTO_LINE_1_TO_I(AutonomousConstants.LinkedWaypoints.AUTO_LINE_1, AutonomousConstants.LinkedWaypoints.I, Optional.of(Branch.I)),
	AUTO_LINE_1_TO_J(AutonomousConstants.LinkedWaypoints.AUTO_LINE_1, AutonomousConstants.LinkedWaypoints.J, Optional.of(Branch.J)),
	AUTO_LINE_1_TO_K(AutonomousConstants.LinkedWaypoints.AUTO_LINE_1, AutonomousConstants.LinkedWaypoints.K, Optional.of(Branch.K)),
	AUTO_LINE_2_TO_H(AutonomousConstants.LinkedWaypoints.AUTO_LINE_2, AutonomousConstants.LinkedWaypoints.H, Optional.of(Branch.H)),
	AUTO_LINE_2_TO_I(AutonomousConstants.LinkedWaypoints.AUTO_LINE_2, AutonomousConstants.LinkedWaypoints.I, Optional.of(Branch.I)),
	AUTO_LINE_2_TO_J(AutonomousConstants.LinkedWaypoints.AUTO_LINE_2, AutonomousConstants.LinkedWaypoints.J, Optional.of(Branch.J)),
	AUTO_LINE_2_TO_K(AutonomousConstants.LinkedWaypoints.AUTO_LINE_2, AutonomousConstants.LinkedWaypoints.K, Optional.of(Branch.K)),
	AUTO_LINE_3_TO_G(AutonomousConstants.LinkedWaypoints.AUTO_LINE_3, AutonomousConstants.LinkedWaypoints.G, Optional.of(Branch.G)),
	AUTO_LINE_3_TO_H(AutonomousConstants.LinkedWaypoints.AUTO_LINE_3, AutonomousConstants.LinkedWaypoints.H, Optional.of(Branch.H)),
	AUTO_LINE_3_TO_I(AutonomousConstants.LinkedWaypoints.AUTO_LINE_3, AutonomousConstants.LinkedWaypoints.I, Optional.of(Branch.I)),
	AUTO_LINE_3_TO_J(AutonomousConstants.LinkedWaypoints.AUTO_LINE_3, AutonomousConstants.LinkedWaypoints.J, Optional.of(Branch.J)),
	AUTO_LINE_4_TO_F(AutonomousConstants.LinkedWaypoints.AUTO_LINE_4, AutonomousConstants.LinkedWaypoints.F, Optional.of(Branch.F)),
	AUTO_LINE_4_TO_G(AutonomousConstants.LinkedWaypoints.AUTO_LINE_4, AutonomousConstants.LinkedWaypoints.G, Optional.of(Branch.G)),
	AUTO_LINE_4_TO_H(AutonomousConstants.LinkedWaypoints.AUTO_LINE_4, AutonomousConstants.LinkedWaypoints.H, Optional.of(Branch.H)),
	AUTO_LINE_4_TO_I(AutonomousConstants.LinkedWaypoints.AUTO_LINE_4, AutonomousConstants.LinkedWaypoints.I, Optional.of(Branch.I)),
	AUTO_LINE_5_TO_E(AutonomousConstants.LinkedWaypoints.AUTO_LINE_5, AutonomousConstants.LinkedWaypoints.E, Optional.of(Branch.E)),
	AUTO_LINE_5_TO_F(AutonomousConstants.LinkedWaypoints.AUTO_LINE_5, AutonomousConstants.LinkedWaypoints.F, Optional.of(Branch.F)),
	AUTO_LINE_5_TO_G(AutonomousConstants.LinkedWaypoints.AUTO_LINE_5, AutonomousConstants.LinkedWaypoints.G, Optional.of(Branch.G)),
	AUTO_LINE_5_TO_I(AutonomousConstants.LinkedWaypoints.AUTO_LINE_5, AutonomousConstants.LinkedWaypoints.I, Optional.of(Branch.I)),
	AUTO_LINE_6_TO_D(AutonomousConstants.LinkedWaypoints.AUTO_LINE_6, AutonomousConstants.LinkedWaypoints.D, Optional.of(Branch.D)),
	AUTO_LINE_6_TO_E(AutonomousConstants.LinkedWaypoints.AUTO_LINE_6, AutonomousConstants.LinkedWaypoints.E, Optional.of(Branch.E)),
	AUTO_LINE_6_TO_F(AutonomousConstants.LinkedWaypoints.AUTO_LINE_6, AutonomousConstants.LinkedWaypoints.F, Optional.of(Branch.F)),
	AUTO_LINE_7_TO_D(AutonomousConstants.LinkedWaypoints.AUTO_LINE_7, AutonomousConstants.LinkedWaypoints.D, Optional.of(Branch.D)),
	AUTO_LINE_7_TO_E(AutonomousConstants.LinkedWaypoints.AUTO_LINE_7, AutonomousConstants.LinkedWaypoints.E, Optional.of(Branch.E)),
	AUTO_LINE_7_TO_F(AutonomousConstants.LinkedWaypoints.AUTO_LINE_7, AutonomousConstants.LinkedWaypoints.F, Optional.of(Branch.F)),
	A_TO_UPPER_CORAL_STATION_2(
		AutonomousConstants.LinkedWaypoints.A,
		AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION_2,
		Optional.empty()
	),
	A_TO_LOWER_CORAL_STATION_2(
		AutonomousConstants.LinkedWaypoints.A,
		AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION_2,
		Optional.empty()
	),
	B_TO_LOWER_CORAL_STATION_2(
		AutonomousConstants.LinkedWaypoints.B,
		AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION_2,
		Optional.empty()
	),
	B_TO_UPPER_CORAL_STATION_2(
		AutonomousConstants.LinkedWaypoints.B,
		AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION_2,
		Optional.empty()
	),
	C_TO_LOWER_CORAL_STATION_2(
		AutonomousConstants.LinkedWaypoints.C,
		AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION_2,
		Optional.empty()
	),
	D_TO_LOWER_CORAL_STATION_2(
		AutonomousConstants.LinkedWaypoints.D,
		AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION_2,
		Optional.empty()
	),
	E_TO_LOWER_CORAL_STATION_2(
		AutonomousConstants.LinkedWaypoints.E,
		AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION_2,
		Optional.empty()
	),
	F_TO_LOWER_CORAL_STATION_2(
		AutonomousConstants.LinkedWaypoints.F,
		AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION_2,
		Optional.empty()
	),
	G_TO_LOWER_CORAL_STATION_2(
		AutonomousConstants.LinkedWaypoints.G,
		AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION_2,
		Optional.empty()
	),
	H_TO_UPPER_CORAL_STATION_2(
		AutonomousConstants.LinkedWaypoints.H,
		AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION_2,
		Optional.empty()
	),
	I_TO_UPPER_CORAL_STATION_2(
		AutonomousConstants.LinkedWaypoints.I,
		AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION_2,
		Optional.empty()
	),
	J_TO_UPPER_CORAL_STATION_2(
		AutonomousConstants.LinkedWaypoints.J,
		AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION_2,
		Optional.empty()
	),
	K_TO_UPPER_CORAL_STATION_2(
		AutonomousConstants.LinkedWaypoints.K,
		AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION_2,
		Optional.empty()
	),
	L_TO_UPPER_CORAL_STATION_2(
		AutonomousConstants.LinkedWaypoints.L,
		AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION_2,
		Optional.empty()
	),
	LOWER_CORAL_STATION_2_TO_A(
		AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION_2,
		AutonomousConstants.LinkedWaypoints.A,
		Optional.of(Branch.A)
	),
	LOWER_CORAL_STATION_2_TO_B(
		AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION_2,
		AutonomousConstants.LinkedWaypoints.B,
		Optional.of(Branch.B)
	),
	LOWER_CORAL_STATION_2_TO_C(
		AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION_2,
		AutonomousConstants.LinkedWaypoints.C,
		Optional.of(Branch.C)
	),
	LOWER_CORAL_STATION_2_TO_D(
		AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION_2,
		AutonomousConstants.LinkedWaypoints.D,
		Optional.of(Branch.D)
	),
	LOWER_CORAL_STATION_2_TO_E(
		AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION_2,
		AutonomousConstants.LinkedWaypoints.E,
		Optional.of(Branch.E)
	),
	LOWER_CORAL_STATION_2_TO_F(
		AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION_2,
		AutonomousConstants.LinkedWaypoints.F,
		Optional.of(Branch.F)
	),
	LOWER_CORAL_STATION_2_TO_G(
		AutonomousConstants.LinkedWaypoints.LOWER_CORAL_STATION_2,
		AutonomousConstants.LinkedWaypoints.G,
		Optional.of(Branch.G)
	),
	UPPER_CORAL_STATION_2_TO_A(
		AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION_2,
		AutonomousConstants.LinkedWaypoints.A,
		Optional.of(Branch.A)
	),
	UPPER_CORAL_STATION_2_TO_B(
		AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION_2,
		AutonomousConstants.LinkedWaypoints.B,
		Optional.of(Branch.B)
	),
	UPPER_CORAL_STATION_2_TO_H(
		AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION_2,
		AutonomousConstants.LinkedWaypoints.H,
		Optional.of(Branch.H)
	),
	UPPER_CORAL_STATION_2_TO_I(
		AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION_2,
		AutonomousConstants.LinkedWaypoints.I,
		Optional.of(Branch.I)
	),
	UPPER_CORAL_STATION_2_TO_J(
		AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION_2,
		AutonomousConstants.LinkedWaypoints.J,
		Optional.of(Branch.J)
	),
	UPPER_CORAL_STATION_2_TO_K(
		AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION_2,
		AutonomousConstants.LinkedWaypoints.K,
		Optional.of(Branch.K)
	),
	UPPER_CORAL_STATION_2_TO_L(
		AutonomousConstants.LinkedWaypoints.UPPER_CORAL_STATION_2,
		AutonomousConstants.LinkedWaypoints.L,
		Optional.of(Branch.L)
	),
	ALGAE_REMOVE_D_TO_FIRST_NET(
		AutonomousConstants.LinkedWaypoints.ALGAE_REMOVE_D,
		AutonomousConstants.LinkedWaypoints.LEFT_NET,
		Optional.empty()
	),
	LEFT_NET_TO_ALGAE_REMOVE_E(
		AutonomousConstants.LinkedWaypoints.LEFT_NET,
		AutonomousConstants.LinkedWaypoints.ALGAE_REMOVE_E,
		Optional.empty()
	),
	ALGAE_REMOVE_E_TO_MIDDLE_NET(
		AutonomousConstants.LinkedWaypoints.ALGAE_REMOVE_E,
		AutonomousConstants.LinkedWaypoints.MIDDLE_NET,
		Optional.empty()
	),
	MIDDLE_NET_TO_ALGAE_REMOVE_C(
		AutonomousConstants.LinkedWaypoints.MIDDLE_NET,
		AutonomousConstants.LinkedWaypoints.ALGAE_REMOVE_C,
		Optional.empty()
	),
	ALGAE_REMOVE_C_TO_RIGHT_NET(
		AutonomousConstants.LinkedWaypoints.ALGAE_REMOVE_C,
		AutonomousConstants.LinkedWaypoints.RIGHT_NET,
		Optional.empty()
	);

	private final Pair<String, Pose2d> startingPoint;
	private final Pair<String, Pose2d> endPoint;
	private final Optional<Branch> targetBranch;

	AutoPath(Pair<String, Pose2d> startingPoint, Pair<String, Pose2d> endPoint, Optional<Branch> targetBranch) {
		this.startingPoint = startingPoint;
		this.endPoint = endPoint;
		this.targetBranch = targetBranch;
	}

	public Pair<String, Pose2d> getStartingPoint() {
		return startingPoint;
	}

	public Pair<String, Pose2d> getEndPoint() {
		return endPoint;
	}

	public Optional<Branch> getTargetBranch() {
		return targetBranch;
	}

	public String getPathName() {
		return startingPoint.getFirst() + "-" + endPoint.getFirst();
	}

	public Optional<PathPlannerPath> getPath() {
		return PathPlannerUtil.getPathFromFile(getPathName());
	}

}
