package frc.robot.scoringhelpers;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import frc.constants.field.Field;
import frc.constants.field.enums.Branch;
import frc.robot.statemachine.StateMachineConstants;

import java.util.HashMap;

public class ScoringPathsHelper {

	private static final HashMap<Branch, PathPlannerPath> BRANCH_PATH_PLANNER_PATH_HASH_MAP = generateAllPaths();

	private static HashMap<Branch, PathPlannerPath> generateAllPaths() {
		HashMap<Branch, PathPlannerPath> branchToPathMap = new HashMap<>();

		Branch[] branches = Branch.values();
		for (Branch branch : branches) {
			branchToPathMap.put(branch, generatePathToTargetBranch(branch));
		}
		return branchToPathMap;
	}


	public static PathPlannerPath getPathByBranch(Branch branch) {
		return BRANCH_PATH_PLANNER_PATH_HASH_MAP.get(branch);
	}

	private static PathPlannerPath generatePathToTargetBranch(Branch branch) {
		return new PathPlannerPath(
			PathPlannerPath.waypointsFromPoses(
				ScoringHelpers.getRobotBranchScoringPose(branch, StateMachineConstants.DISTANCE_TO_BRANCH_FOR_STARTING_PATH),
				ScoringHelpers.getRobotBranchScoringPose(branch, StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS)
			),
			new PathConstraints(
				StateMachineConstants.MAX_VELOCITY_WHILE_ELEVATOR_L4_METERS_PER_SECOND,
				StateMachineConstants.MAX_ACCELERATION_WHILE_ELEVATOR_L4_METERS_PER_SECOND_SQUARED,
				StateMachineConstants.MAX_VELOCITY_WHILE_ELEVATOR_L4_ROTATION2D_PER_SECOND.getRadians(),
				StateMachineConstants.MAX_ACCELERATION_WHILE_ELEVATOR_L4_ROTATION2D_PER_SECOND_SQUARED.getRadians()
			),
			new IdealStartingState(
				StateMachineConstants.MAX_VELOCITY_WHILE_ELEVATOR_L4_METERS_PER_SECOND,
				Field.getReefSideMiddle(branch.getReefSide()).getRotation()
			),
			new GoalEndState(0, Field.getReefSideMiddle(branch.getReefSide()).getRotation())
		);
	}


}
