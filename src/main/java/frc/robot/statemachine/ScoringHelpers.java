package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Pose2d;
import frc.constants.field.Tower;

public class ScoringHelpers {

	public static Tower getClosestTower(Pose2d robotPose) {
		Tower smallestDistanceTower = Tower.CLOSE_TOWER;
		for (Tower tower : Tower.values()) {
			if (
				robotPose.getTranslation().getDistance(smallestDistanceTower.getTower())
					< robotPose.getTranslation().getDistance(tower.getTower())
			)
				smallestDistanceTower = tower;
		}
		return smallestDistanceTower;
	}

}
