package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Pose2d;
import frc.constants.field.Tower;

public class ScoringHelpers {

	public static Tower getClosestTower(Pose2d robotPose) {
		Tower smallestDistanceTower = Tower.CLOSE_TOWER;
		for (Tower tower : Tower.values()) {
			if (
				robotPose.getTranslation().getDistance(smallestDistanceTower.getTower().getTranslation())
					> robotPose.getTranslation().getDistance(tower.getTower().getTranslation())
			)
				smallestDistanceTower = tower;
		}
		return smallestDistanceTower;
	}

	public static double getDistanceFromTower(Tower tower, Pose2d robotPose) {
		return tower.getTower().getTranslation().getDistance(robotPose.getTranslation());
	}

	public static double getDistanceFromClosestTower(Pose2d robotPose) {
		return getDistanceFromTower(getClosestTower(robotPose), robotPose);
	}

}
