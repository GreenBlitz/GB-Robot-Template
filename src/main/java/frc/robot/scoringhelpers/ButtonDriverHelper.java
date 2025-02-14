package frc.robot.scoringhelpers;

import edu.wpi.first.math.geometry.Pose2d;
import frc.constants.field.Field;
import frc.constants.field.enums.ReefSide;
import org.littletonrobotics.junction.Logger;

public class ButtonDriverHelper {

	private static final double METERS_FROM_REEF_FOR_SIDE_HIGHLIGHTING = 0.45;
	private static final int CHOSEN_SIDE = 0;
	private static final int CHOSEN_SCORE_LEVEL = 0;
	private static final Pose2d[] SCORE_LEVEL_OPTIONS = {new Pose2d()};
	private static final Pose2d[] REEF_SIDES = {
		getPointFromCertainDistance(Field.getReefSideMiddle(ReefSide.A), METERS_FROM_REEF_FOR_SIDE_HIGHLIGHTING),
		getPointFromCertainDistance(Field.getReefSideMiddle(ReefSide.B), METERS_FROM_REEF_FOR_SIDE_HIGHLIGHTING),
		getPointFromCertainDistance(Field.getReefSideMiddle(ReefSide.C), METERS_FROM_REEF_FOR_SIDE_HIGHLIGHTING),
		getPointFromCertainDistance(Field.getReefSideMiddle(ReefSide.D), METERS_FROM_REEF_FOR_SIDE_HIGHLIGHTING),
		getPointFromCertainDistance(Field.getReefSideMiddle(ReefSide.E), METERS_FROM_REEF_FOR_SIDE_HIGHLIGHTING),
		getPointFromCertainDistance(Field.getReefSideMiddle(ReefSide.F), METERS_FROM_REEF_FOR_SIDE_HIGHLIGHTING)};


	public static Pose2d[] getHighlightReef(ReefSide reefSide) {
		Pose2d[] highlightReef = new Pose2d[REEF_SIDES.length];
		highlightReef[CHOSEN_SIDE] = REEF_SIDES[reefSide.getIndex()];

		for (int darkSide = 1, reefSideIndex = 0; reefSideIndex < REEF_SIDES.length; reefSideIndex++) {
			if (reefSide.getIndex() != reefSideIndex) {
				highlightReef[darkSide] = REEF_SIDES[reefSideIndex];
				darkSide++;
			}
		}
		return highlightReef;
	}

	public static Pose2d getPointFromCertainDistance(Pose2d point, double distantInMeters) {
		return new Pose2d(
			point.getX() - point.getRotation().getCos() * distantInMeters,
			point.getY() - point.getRotation().getSin() * distantInMeters,
			point.getRotation()
		);
	}


	public static void log(String logPath) {
		Pose2d[] highlightReef = getHighlightReef(ScoringHelpers.getTargetReefSide());

		Logger.recordOutput(logPath + "/ChosenReefSide", highlightReef[CHOSEN_SIDE]);
		for (int index = 1; index < highlightReef.length; index++) {
			Logger.recordOutput(logPath + "/DarkReefSide" + index, highlightReef[index]);
		}
	}


}
