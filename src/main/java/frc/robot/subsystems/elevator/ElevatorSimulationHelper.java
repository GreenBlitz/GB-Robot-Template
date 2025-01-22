package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class ElevatorSimulationHelper {

	public static Pose3d getFirstStagePose(double heightMeters) {
		if (heightMeters > ElevatorConstants.FIRST_STAGE_MAXIMUM_HEIGHT_METERS) {
			return getPose3dFromHeight(heightMeters - ElevatorConstants.FIRST_STAGE_MAXIMUM_HEIGHT_METERS);
		}
		return getPose3dFromHeight(0);
	}

	public static Pose3d getSecondStagePose(double heightMeters) {
		return getPose3dFromHeight(heightMeters);
	}

	private static Pose3d getPose3dFromHeight(double heightMeters) {
		return new Pose3d(new Translation3d(0, 0, heightMeters), new Rotation3d());
	}

}
