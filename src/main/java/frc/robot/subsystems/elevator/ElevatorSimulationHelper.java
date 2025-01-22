package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class ElevatorSimulationHelper {


	public static Pose3d getFirstElevatorStagePose(double heightInMeters) {
		if (heightInMeters > ElevatorConstants.MAXIMUM_FIRST_STAGE_HEIGHT_METERS) {
			return get3dPoseFromHeight(heightInMeters - ElevatorConstants.MAXIMUM_FIRST_STAGE_HEIGHT_METERS);
		}
		return get3dPoseFromHeight(0);
	}

	public static Pose3d getSecondElevatorStagePose(double heightInMeters) {
		return get3dPoseFromHeight(heightInMeters);
	}

	private static Pose3d get3dPoseFromHeight(double heightInMeters) {
		return new Pose3d(new Translation3d(0, 0, heightInMeters), new Rotation3d());
	}

}
