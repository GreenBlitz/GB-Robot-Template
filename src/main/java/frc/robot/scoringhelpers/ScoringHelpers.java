package frc.robot.scoringhelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.constants.field.Field;
import frc.constants.field.enums.Branch;
import frc.constants.field.enums.CoralStation;
import frc.constants.field.enums.ReefSide;
import frc.robot.Robot;
import frc.robot.statemachine.RobotState;
import frc.robot.statemachine.superstructure.ScoreLevel;
import frc.utils.Side;
import org.littletonrobotics.junction.Logger;

import java.util.Set;

public class ScoringHelpers {

	public static CoralStation targetCoralStation = CoralStation.LEFT;
	public static ScoreLevel targetScoreLevel = ScoreLevel.L1;

	public static ReefSide getTargetReefSide() {
		return ReefSide.getReefSideBySideAndFar(targetReefSide, isFarReef);
	}

	public static Branch getTargetBranch() {
		return Branch.getBranchByReefSideAndSide(getTargetReefSide(), isLeftBranch);
	}


	private static boolean isFarReef = false;
	private static Side targetReefSide = Side.MIDDLE;
	private static boolean isLeftBranch = false;

	public static void toggleFarReefSide() {
		isFarReef = !isFarReef;
	}

	public static void toggleLeftBranch() {
		isLeftBranch = !isLeftBranch;
	}

	public static void setTargetReefSide(Side side) {
		targetReefSide = side;
	}

	public static void toggleTargetCoralStation() {
		targetCoralStation = targetCoralStation == CoralStation.LEFT ? CoralStation.RIGHT : CoralStation.LEFT;
	}


	public static Pose2d getRobotScoringPose(Branch branch, double distanceFromBranchMeters) {
		Translation2d branchTranslation = Field.getCoralPlacement(branch);
		Rotation2d targetRobotAngle = Field.getReefSideMiddle(branch.getReefSide()).getRotation();
		Translation2d differenceTranslation = new Translation2d(distanceFromBranchMeters, targetRobotAngle);
		return new Pose2d(branchTranslation.minus(differenceTranslation), targetRobotAngle);
	}

	public static Command scoreToChosenScoreLevel(Robot robot) {
		return new DeferredCommand(
			() -> switch (targetScoreLevel) {
				case L1 -> robot.getRobotCommander().setState(RobotState.L1);
				case L2 -> robot.getRobotCommander().setState(RobotState.L2);
				case L3 -> robot.getRobotCommander().setState(RobotState.L3);
				case L4 -> robot.getRobotCommander().setState(RobotState.L4);
			},
			Set.of(
				robot.getRobotCommander(),
				robot.getSwerve(),
				robot.getRobotCommander().getSuperstructure(),
				robot.getArm(),
				robot.getEndEffector(),
				robot.getElevator()
			)
		);
	}

	public static Command preScoreToChosenScoreLevel(Robot robot) {
		return new DeferredCommand(
			() -> switch (targetScoreLevel) {
				case L1 -> robot.getRobotCommander().setState(RobotState.PRE_L1);
				case L2 -> robot.getRobotCommander().setState(RobotState.PRE_L2);
				case L3 -> robot.getRobotCommander().setState(RobotState.PRE_L3);
				case L4 -> robot.getRobotCommander().setState(RobotState.PRE_L4);
			},
			Set.of(
				robot.getRobotCommander(),
				robot.getSwerve(),
				robot.getRobotCommander().getSuperstructure(),
				robot.getArm(),
				robot.getEndEffector(),
				robot.getElevator()
			)
		);
	}

	public static void log(String logPath) {
		Logger.recordOutput(logPath + "/TargetBranch", getTargetBranch());
		Logger.recordOutput(logPath + "/TargetReefSide", getTargetReefSide());
		Logger.recordOutput(logPath + "/TargetCoralStation", targetCoralStation);
		Logger.recordOutput(logPath + "/TargetScoreLevel", targetScoreLevel);
	}


}
