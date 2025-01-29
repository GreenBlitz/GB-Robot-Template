package frc.robot.scoringhelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.Field;
import frc.constants.field.enums.ReefBranch;

public class ScoringHelpers {

	public static Pose2d getRobotScoringPose(ReefBranch branch, double distanceFromBranchMeters) {
		Translation2d branchTranslation = Field.getCoralPlacement(branch);
		Rotation2d targetRobotAngle = Field.getMiddleOfReefSide(branch.getReefSide()).getRotation();

		double xDistanceFromBranch = distanceFromBranchMeters * Math.cos(targetRobotAngle.getRadians());
		double yDistanceFromBranch = distanceFromBranchMeters * Math.sin(targetRobotAngle.getRadians());

		Translation2d scoringTranslation = new Translation2d(
			branchTranslation.getX() - xDistanceFromBranch,
			branchTranslation.getY() - yDistanceFromBranch
		);
		return new Pose2d(scoringTranslation, targetRobotAngle);
	}


}
