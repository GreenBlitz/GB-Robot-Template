package frc.robot.scoringhelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.Field;
import frc.constants.field.enums.ReefBranch;

public class ScoringHelpers {

	public static Pose2d getRobotScoringPose(ReefBranch branch, double distanceFromBranchMeters) {
		Translation2d branchTranslation = Field.getCoralPlacement(branch);
		Rotation2d robotAngle = Field.getMiddleOfReefSide(branch.getReefSide()).getRotation();

		double xDiff = distanceFromBranchMeters * Math.cos(robotAngle.getRadians());
		double yDiff = distanceFromBranchMeters * Math.sin(robotAngle.getRadians());

		Translation2d scoringTrans = new Translation2d(branchTranslation.getX() - xDiff, branchTranslation.getY() - yDiff);
		return new Pose2d(scoringTrans, robotAngle);
	}


}
