package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Pose2d;
import frc.constants.field.Tower;
import frc.robot.statemachine.shooterstatehandler.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class ScoringHelpers {

	public static Tower getClosestTower(Pose2d robotPose) {
		Tower smallestDistanceTower = Tower.CLOSE_TOWER;
		for (Tower tower : Tower.values()) {
			if (
				robotPose.getTranslation().getDistance(smallestDistanceTower.getPose().getTranslation())
					> robotPose.getTranslation().getDistance(tower.getPose().getTranslation())
			)
				smallestDistanceTower = tower;
		}
		Logger.recordOutput(ShooterConstants.LOG_PATH + "/ClosestTower", smallestDistanceTower.getPose());
		return smallestDistanceTower;
	}

	public static double getDistanceFromClosestTower(Pose2d robotPose) {
		return getClosestTower(robotPose).getPose().getTranslation().getDistance(robotPose.getTranslation());
	}

}
