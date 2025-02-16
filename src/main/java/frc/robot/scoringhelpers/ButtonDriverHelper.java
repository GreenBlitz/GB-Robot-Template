package frc.robot.scoringhelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.field.Field;
import frc.constants.field.enums.Branch;
import frc.constants.field.enums.ReefSide;
import frc.robot.statemachine.superstructure.ScoreLevel;
import frc.utils.math.AngleTransform;
import org.littletonrobotics.junction.Logger;

public class ButtonDriverHelper {

	private static final double METERS_FROM_REEF_FOR_SIDE_HIGHLIGHTING = 0.45;
	private static final Pose2d L1_DISPLAY_PLACEMENT = Field
		.getAllianceRelative(new Pose2d(3.22, 6.85, Rotation2d.fromDegrees(0)), true, true, AngleTransform.INVERT);
	private static final Pose2d L2_DISPLAY_PLACEMENT = Field
		.getAllianceRelative(new Pose2d(4.55, 6.85, Rotation2d.fromDegrees(0)), true, true, AngleTransform.INVERT);
	private static final Pose2d L3_DISPLAY_PLACEMENT = Field
		.getAllianceRelative(new Pose2d(5.88, 6.85, Rotation2d.fromDegrees(0)), true, true, AngleTransform.INVERT);
	private static final Pose2d L4_DISPLAY_PLACEMENT = Field
		.getAllianceRelative(new Pose2d(7.22, 6.85, Rotation2d.fromDegrees(0)), true, true, AngleTransform.INVERT);
	private static final Pose2d LEFT_TOGGLE_PLACEMENT = Field
		.getAllianceRelative(new Pose2d(7.22, 4.55, Rotation2d.fromDegrees(0)), true, true, AngleTransform.INVERT);
	private static final Pose2d RIGHT_TOGGLE_PLACEMENT = Field
		.getAllianceRelative(new Pose2d(7.22, 3.55, Rotation2d.fromDegrees(0)), true, true, AngleTransform.INVERT);
	private static final Pose2d HIDDEN_PLACEMENT = new Pose2d(-10, -10, Rotation2d.fromDegrees(90));
	private static final int CHOSEN_SIDE = 0;
	private static final int CHOSEN_SCORE_LEVEL = 0;
	private static final int LEFT_INDEX = 0;
	private static final int RIGHT_INDEX = 1;
	private static final int EMPTY_TOGGLE = 2;
	private static final Pose2d[] REEF_SIDES = {
		getPointFromCertainDistance(Field.getReefSideMiddle(ReefSide.A), METERS_FROM_REEF_FOR_SIDE_HIGHLIGHTING),
		getPointFromCertainDistance(Field.getReefSideMiddle(ReefSide.B), METERS_FROM_REEF_FOR_SIDE_HIGHLIGHTING),
		getPointFromCertainDistance(Field.getReefSideMiddle(ReefSide.C), METERS_FROM_REEF_FOR_SIDE_HIGHLIGHTING),
		getPointFromCertainDistance(Field.getReefSideMiddle(ReefSide.D), METERS_FROM_REEF_FOR_SIDE_HIGHLIGHTING),
		getPointFromCertainDistance(Field.getReefSideMiddle(ReefSide.E), METERS_FROM_REEF_FOR_SIDE_HIGHLIGHTING),
		getPointFromCertainDistance(Field.getReefSideMiddle(ReefSide.F), METERS_FROM_REEF_FOR_SIDE_HIGHLIGHTING)};
	private static final Pose2d[] SCORE_LEVEL_PLACEMENTS = {
		L1_DISPLAY_PLACEMENT,
		L2_DISPLAY_PLACEMENT,
		L3_DISPLAY_PLACEMENT,
		L4_DISPLAY_PLACEMENT};

	public static Pose2d[] getReefDisplay(ReefSide reefSide) {
		Pose2d[] reefDisplay = new Pose2d[REEF_SIDES.length];
		reefDisplay[CHOSEN_SIDE] = REEF_SIDES[reefSide.getIndex()];

		for (int darkSide = 1, reefSideIndex = 0; reefSideIndex < REEF_SIDES.length; reefSideIndex++) {
			if (reefSide.getIndex() != reefSideIndex) {
				reefDisplay[darkSide] = REEF_SIDES[reefSideIndex];
				darkSide++;
			}
		}
		return reefDisplay;
	}

	public static Pose2d[] getScoreLevelDisplay(ScoreLevel scoreLevel) {
		Pose2d[] scoreLevelDisplay = new Pose2d[SCORE_LEVEL_PLACEMENTS.length];
		for (int darkLevel = 1, index = 0; index < SCORE_LEVEL_PLACEMENTS.length; index++) {
			if (index == scoreLevel.ordinal()) {
				scoreLevelDisplay[CHOSEN_SCORE_LEVEL] = SCORE_LEVEL_PLACEMENTS[index];
			} else {
				scoreLevelDisplay[darkLevel] = SCORE_LEVEL_PLACEMENTS[index];
				darkLevel++;
			}
		}
		return scoreLevelDisplay;
	}

	public static Pose2d[] getLeftRightToggleDisplay(Branch targetBranch) {
		return targetBranch.isLeft()
			? new Pose2d[] {LEFT_TOGGLE_PLACEMENT, HIDDEN_PLACEMENT, RIGHT_TOGGLE_PLACEMENT}
			: new Pose2d[] {HIDDEN_PLACEMENT, RIGHT_TOGGLE_PLACEMENT, LEFT_TOGGLE_PLACEMENT};
	}

	public static Pose2d getPointFromCertainDistance(Pose2d point, double distantInMeters) {
		return new Pose2d(
			point.getX() - point.getRotation().getCos() * distantInMeters,
			point.getY() - point.getRotation().getSin() * distantInMeters,
			point.getRotation()
		);
	}

	public static void log(String logPath) {
		Pose2d[] reefDisplay = getReefDisplay(ScoringHelpers.getTargetReefSide());
		Pose2d[] scoreLevelDisplay = getScoreLevelDisplay(ScoringHelpers.targetScoreLevel);
		Pose2d[] leftRightToggleDisplay = getLeftRightToggleDisplay(ScoringHelpers.getTargetBranch());

		Logger.recordOutput(logPath + "/ChosenReefSide", reefDisplay[CHOSEN_SIDE]);
		for (int index = 1; index < reefDisplay.length; index++) {
			Logger.recordOutput(logPath + "/DarkReefSide" + index, reefDisplay[index]);
		}

		Logger.recordOutput(logPath + "/ChosenScoreLevel", scoreLevelDisplay[CHOSEN_SCORE_LEVEL]);
		for (int index = 1; index < scoreLevelDisplay.length; index++) {
			Logger.recordOutput(logPath + "/DarkScoreLevel" + index, scoreLevelDisplay[index]);
		}

		Logger.recordOutput(logPath + "/IsLeft", leftRightToggleDisplay[LEFT_INDEX]);
		Logger.recordOutput(logPath + "/IsRight", leftRightToggleDisplay[RIGHT_INDEX]);
		Logger.recordOutput(logPath + "/EmptyToggle", leftRightToggleDisplay[EMPTY_TOGGLE]);
	}


}
