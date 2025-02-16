package frc.robot.scoringhelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.Field;
import frc.constants.field.enums.Branch;
import frc.constants.field.enums.CoralStationSlot;

public class ScoringHelpers {

	public static Branch targetBranch = Branch.C;

	public static Pose2d getRobotScoringPose(Branch branch, double distanceFromBranchMeters) {
		Translation2d branchTranslation = Field.getCoralPlacement(branch);
		Rotation2d targetRobotAngle = Field.getReefSideMiddle(branch.getReefSide()).getRotation();
		Translation2d differenceTranslation = new Translation2d(distanceFromBranchMeters, targetRobotAngle);
		return new Pose2d(branchTranslation.minus(differenceTranslation), targetRobotAngle);
	}
	public static Pose2d getRobotFeederPose(CoralStationSlot coralStationSlot, double distanceFromSlots) {
		Translation2d coralStationTranslation = Field.getCoralStationTranslation2d(coralStationSlot);
		Rotation2d targetRobotAngle = Field.getCoralStationSlotsPose2d(coralStationSlot).getRotation();
		Translation2d differenceTranslation = new Translation2d(distanceFromSlots, targetRobotAngle);
		return new Pose2d(coralStationTranslation.minus(differenceTranslation), targetRobotAngle);
	}
}
