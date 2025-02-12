package frc.robot.scoringhelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.constants.field.Field;
import frc.constants.field.enums.Branch;
import frc.constants.field.enums.CoralStation;
import frc.constants.field.enums.ReefSide;
import frc.robot.Robot;
import frc.robot.statemachine.RobotState;
import frc.robot.statemachine.superstructure.ScoreLevel;
import frc.utils.pose.Side;
import org.littletonrobotics.junction.Logger;

import java.util.Map;

public class ScoringHelpers {

	public static ScoreLevel targetScoreLevel = ScoreLevel.L1;

	private static final Translation2d LEFT_CORAL_STATION_TRANSLATION = Field.getCoralStationMiddle(CoralStation.LEFT).getTranslation();
	private static final Translation2d RIGHT_CORAL_STATION_TRANSLATION = Field.getCoralStationMiddle(CoralStation.RIGHT).getTranslation();

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
		return new Pose2d(branchTranslation.minus(differenceTranslation), targetRobotAngle);
	}


	public static Command scoreToChosenScoreLevel(Robot robot) {
		return new SelectCommand<>(
			Map.of(
				ScoreLevel.L1,
				robot.getRobotCommander().score(ScoreLevel.L1),
				ScoreLevel.L2,
				robot.getRobotCommander().score(ScoreLevel.L2),
				ScoreLevel.L3,
				robot.getRobotCommander().score(ScoreLevel.L3),
				ScoreLevel.L4,
				robot.getRobotCommander().score(ScoreLevel.L4)
			),
			() -> targetScoreLevel
		);
	}

	public static Command preScoreToChosenScoreLevel(Robot robot) {
		return new SelectCommand<>(
			Map.of(
				ScoreLevel.L1,
				robot.getRobotCommander().fullyPreScore(ScoreLevel.L1),
				ScoreLevel.L2,
				robot.getRobotCommander().fullyPreScore(ScoreLevel.L2),
				ScoreLevel.L3,
				robot.getRobotCommander().fullyPreScore(ScoreLevel.L3),
				ScoreLevel.L4,
				robot.getRobotCommander().fullyPreScore(ScoreLevel.L4)
			),
			() -> targetScoreLevel
		);
	}

	public static void log(String logPath) {
		Logger.recordOutput(logPath + "/TargetBranch", getTargetBranch());
		Logger.recordOutput(logPath + "/TargetReefSide", getTargetReefSide());
		Logger.recordOutput(logPath + "/TargetCoralStation", latestWantedCoralStation);
		Logger.recordOutput(logPath + "/TargetScoreLevel", targetScoreLevel);
	}


}
