package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Pose2d;
import frc.constants.field.TowerSide;
import org.littletonrobotics.junction.Logger;

public class ScoringHelpers {

	public static TowerSide getClosestTower(Pose2d pose) {
		TowerSide smallestDistanceTower = TowerSide.CLOSE_TOWER;
		for (TowerSide tower : TowerSide.values()) {
			if (
				pose.getTranslation().getDistance(smallestDistanceTower.getPose().getTranslation())
					> pose.getTranslation().getDistance(tower.getPose().getTranslation())
			)
				smallestDistanceTower = tower;
		}
		Logger.recordOutput("ScoringHelpers/ClosestTower", smallestDistanceTower.getPose());
		return smallestDistanceTower;
	}

	public static double getDistanceFromClosestTower(Pose2d pose) {
		return getClosestTower(pose).getPose().getTranslation().getDistance(pose.getTranslation());
	}

}
