package frc.robot.scoringhelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.Field;
import frc.constants.field.enums.Branch;
import frc.constants.field.enums.CoralStation;
import frc.constants.field.enums.ReefSide;
import frc.robot.Robot;
import frc.robot.statemachine.superstructure.ScoreLevel;
import frc.utils.pose.Side;
import org.littletonrobotics.junction.Logger;

public class ScoringHelpers {

	public static final Translation2d END_EFFECTOR_OFFSET_FROM_MID_ROBOT = new Translation2d(0, 0.014);

	public static ScoreLevel targetScoreLevel = ScoreLevel.L4;

	private static boolean isFarReefHalf = false;
	private static Side targetSideForReef = Side.MIDDLE;
	private static boolean isLeftBranch = false;
	private static CoralStation latestWantedCoralStation = CoralStation.LEFT;

	public static ReefSide getTargetReefSide() {
		return ReefSide.getReefSideBySideAndFar(targetSideForReef, isFarReefHalf);
	}

	public static Branch getTargetBranch() {
		return Branch.getBranchByReefSideAndSide(getTargetReefSide(), isLeftBranch);
	}

	public static CoralStation getTargetCoralStation(Robot robot) {
		Translation2d robotTranslation = robot.getPoseEstimator().getEstimatedPose().getTranslation();
		Translation2d leftCoralStationTranslation = Field.getCoralStationMiddle(CoralStation.LEFT).getTranslation();
		Translation2d rightCoralStationTranslation = Field.getCoralStationMiddle(CoralStation.RIGHT).getTranslation();
		if (robotTranslation.getDistance(leftCoralStationTranslation) < robotTranslation.getDistance(rightCoralStationTranslation)) {
			latestWantedCoralStation = CoralStation.LEFT;
		} else {
			latestWantedCoralStation = CoralStation.RIGHT;
		}
		return latestWantedCoralStation;
	}


	public static void toggleIsFarReefHalf() {
		isFarReefHalf = !isFarReefHalf;
	}

	public static void toggleIsLeftBranch() {
		isLeftBranch = !isLeftBranch;
	}

	public static void setTargetSideForReef(Side side) {
		targetSideForReef = side;
	}


	public static Pose2d getRobotBranchScoringPose(Branch branch, double distanceFromBranchMeters) {
		Translation2d branchTranslation = Field.getCoralPlacement(branch);
		Rotation2d targetRobotAngle = Field.getReefSideMiddle(branch.getReefSide()).getRotation();
		Translation2d differenceTranslation = new Translation2d(distanceFromBranchMeters, targetRobotAngle);
		Translation2d endeffectorOffsetDifference = END_EFFECTOR_OFFSET_FROM_MID_ROBOT.rotateBy(targetRobotAngle);
		return new Pose2d(branchTranslation.minus(differenceTranslation).minus(endeffectorOffsetDifference), targetRobotAngle);
	}

	public static void log(String logPath) {
		Logger.recordOutput(logPath + "/TargetBranch", getTargetBranch());
		Logger.recordOutput(logPath + "/TargetReefSide", getTargetReefSide());
		Logger.recordOutput(logPath + "/TargetCoralStation", latestWantedCoralStation);
		Logger.recordOutput(logPath + "/TargetScoreLevel", targetScoreLevel);
	}

}
