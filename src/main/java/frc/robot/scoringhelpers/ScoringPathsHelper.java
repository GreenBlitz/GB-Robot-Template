package frc.robot.scoringhelpers;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.constants.field.Field;
import frc.constants.field.enums.Branch;
import frc.robot.statemachine.StateMachineConstants;

import java.util.HashMap;

public class ScoringPathsHelper {

    private static final HashMap<Branch, PathPlannerPath> BRANCH_PATH_PLANNER_PATH_HASH_MAP = generateAllPaths();

    public static HashMap<Branch, PathPlannerPath> generateAllPaths(){
        HashMap<Branch, PathPlannerPath> branchToPathMap = new HashMap<>();

        Branch[] branches = Branch.values();
        for (Branch branch : branches){
            branchToPathMap.put(branch, generatePathToTargetBranch(branch));
        }
        return branchToPathMap;
    }

    public static PathPlannerPath getPathByTargetBranch(Branch branch){
        return BRANCH_PATH_PLANNER_PATH_HASH_MAP.get(branch);
    }

    public static PathPlannerPath generatePathToTargetBranch(Branch branch){

        return new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(
                        ScoringHelpers.getRobotBranchScoringPose(branch, StateMachineConstants.OPEN_SUPERSTRUCTURE_DISTANCE_FROM_REEF_METERS),
                        ScoringHelpers.getRobotBranchScoringPose(branch, StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS)
                ),
                new PathConstraints(
                        1.5,
                        1,
                        2.5,
                        1
                ),
                new IdealStartingState(1, Field.getReefSideMiddle(branch.getReefSide()).getRotation()),
                new GoalEndState(0, Field.getReefSideMiddle(branch.getReefSide()).getRotation())
        );
    }





}
