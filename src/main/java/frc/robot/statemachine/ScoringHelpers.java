package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Pose2d;
import frc.constants.field.TowerSide;
import org.littletonrobotics.junction.Logger;

public class ScoringHelpers {

	public static TowerSide getClosestTower(Pose2d turretPose) {
		TowerSide smallestDistanceTower = TowerSide.CLOSE_TOWER;
		for (TowerSide tower : TowerSide.values()) {
			if (
				turretPose.getTranslation().getDistance(smallestDistanceTower.getPose().getTranslation())
					> turretPose.getTranslation().getDistance(tower.getPose().getTranslation())
			)
				smallestDistanceTower = tower;
		}
		Logger.recordOutput("ScoringHelpers/ClosestTower", smallestDistanceTower.getPose());
		return smallestDistanceTower;
	}

	public static double getDistanceFromClosestTower(Pose2d robotPose) {
		return getClosestTower(robotPose).getPose().getTranslation().getDistance(robotPose.getTranslation());
	}

}
