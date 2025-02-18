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
import frc.utils.Conversions;

public class ScoringPathsHelper {

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
