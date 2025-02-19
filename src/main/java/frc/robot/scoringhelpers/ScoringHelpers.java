package frc.robot.scoringhelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.Field;
import frc.constants.field.enums.Branch;
import frc.constants.field.enums.CoralStationSlot;
import frc.constants.field.enums.CoralStation;
import frc.constants.field.enums.ReefSide;
import frc.robot.Robot;
import frc.robot.statemachine.superstructure.ScoreLevel;
import frc.utils.pose.Side;
import org.littletonrobotics.junction.Logger;

public class ScoringHelpers {

	public static CoralStationSlot targetCoralStationSlot = CoralStationSlot.R1;

	public static final Translation2d END_EFFECTOR_OFFSET_FROM_MID_ROBOT = new Translation2d(0, 0);

	private static final Translation2d LEFT_CORAL_STATION_TRANSLATION = Field.getCoralStationMiddle(CoralStation.LEFT).getTranslation();
	private static final Translation2d RIGHT_CORAL_STATION_TRANSLATION = Field.getCoralStationMiddle(CoralStation.RIGHT).getTranslation();
	
	public static ScoreLevel targetScoreLevel = ScoreLevel.L2;
	public static Branch targetBranch = Branch.C;

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
		if (robotTranslation.getDistance(LEFT_CORAL_STATION_TRANSLATION) < robotTranslation.getDistance(RIGHT_CORAL_STATION_TRANSLATION)) {
			latestWantedCoralStation = CoralStation.LEFT;
		} else {
			latestWantedCoralStation = CoralStation.RIGHT;
		}
		return latestWantedCoralStation;
	}
	
	public static CoralStationSlot getTargetCoralStationSlot(Robot robot) {
		Translation2d robotTranslation = robot.getPoseEstimator().getEstimatedPose().getTranslation();
		Translation2d robotTranslationWithOffset = robot.getPoseEstimator().getEstimatedPose().getTranslation().minus(END_EFFECTOR_OFFSET_FROM_MID_ROBOT);
		targetCoralStationSlot = switch (getTargetCoralStation(robot)) {
			case RIGHT -> {
				double distanceFromLeftSlot = robotTranslation.getDistance(CoralStationSlot.R2.getPosition().getTranslation());
				double distanceFromMiddleSlot = robotTranslation.getDistance(CoralStationSlot.R5.getPosition().getTranslation());
				double distanceFromRightSlot = robotTranslation.getDistance(CoralStationSlot.R8.getPosition().getTranslation());
				if (distanceFromLeftSlot < distanceFromMiddleSlot && distanceFromLeftSlot < distanceFromRightSlot) {
					yield CoralStationSlot.R2;
				}
				if (distanceFromMiddleSlot < distanceFromRightSlot) {
					yield CoralStationSlot.R5;
				}
				yield CoralStationSlot.R8;
			}
			case LEFT -> {
				double distanceFromLeftSlot = robotTranslation.getDistance(CoralStationSlot.L2.getPosition().getTranslation());
				double distanceFromMiddleSlot = robotTranslation.getDistance(CoralStationSlot.L5.getPosition().getTranslation());
				double distanceFromRightSlot = robotTranslation.getDistance(CoralStationSlot.L8.getPosition().getTranslation());
				if (distanceFromLeftSlot < distanceFromMiddleSlot && distanceFromLeftSlot < distanceFromRightSlot) {
					yield CoralStationSlot.L2;
				}
				if (distanceFromMiddleSlot < distanceFromRightSlot) {
					yield CoralStationSlot.L5;
				}
				yield CoralStationSlot.L8;
			}
		};
		
		return targetCoralStationSlot;
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
		Logger.recordOutput(logPath + "/TargetCoralStationSlot", targetCoralStationSlot);
		Logger.recordOutput(logPath + "/TargetScoreLevel", targetScoreLevel);
	}

	public static Pose2d getRobotFeederPose(CoralStationSlot coralStationSlot, double distanceFromSlots) {
		Translation2d coralStationTranslation = Field.getCoralStationSlotsPose2d(coralStationSlot).getTranslation();
		Rotation2d targetRobotAngle = Field.getCoralStationSlotsPose2d(coralStationSlot).getRotation();
		Translation2d differenceTranslation = new Translation2d(distanceFromSlots, targetRobotAngle);
		return new Pose2d(coralStationTranslation.minus(differenceTranslation), targetRobotAngle);
	}

}
