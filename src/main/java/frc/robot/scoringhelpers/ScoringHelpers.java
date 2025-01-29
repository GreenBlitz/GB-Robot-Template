package frc.robot.scoringhelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.Field;
import frc.constants.field.enums.ReefBranch;

public class ScoringHelpers {

	private static final double DISTANCE_FROM_BRANCH_METERS = 0.1;

	public static Pose2d getRobotScoringPose(ReefBranch branch) {
		Translation2d branchTranslation = Field.getCoralPlacement(branch);
		Rotation2d robotAngle = Field.getMiddleOfReefSide(branch.getReefSide()).getRotation();

		double xDiff = DISTANCE_FROM_BRANCH_METERS / Math.cos(robotAngle.getRadians());
		double yDiff = DISTANCE_FROM_BRANCH_METERS / Math.sin(robotAngle.getRadians());

		Translation2d scoringTrans = new Translation2d(branchTranslation.getX() - xDiff, branchTranslation.getY() - yDiff);
		return new Pose2d(scoringTrans, robotAngle);
	}


}
