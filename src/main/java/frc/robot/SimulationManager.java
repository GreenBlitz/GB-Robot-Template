package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.elevator.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

public class SimulationManager {

	private final String logPath;
	private final Robot robot;

	public SimulationManager(String logPath, Robot robot) {
		this.logPath = logPath;
		this.robot = robot;
	}

	public void log() {
		logElevatorPosition3d();
	}

	private void logElevatorPosition3d() {
		Logger
			.recordOutput(logPath + "/Elevator/FirstStagePosition", getElevatorFirstStagePosition(robot.getElevator().getElevatorPositionMeters()));
		Logger.recordOutput(
			logPath + "/Elevator/SecondStagePosition",
			getElevatorSecondStagePosition(robot.getElevator().getElevatorPositionMeters())
		);
	}

	public static Pose3d getElevatorFirstStagePosition(double heightMeters) {
		if (heightMeters > ElevatorConstants.FIRST_STAGE_MAXIMUM_HEIGHT_METERS) {
			return getElevatorPose3dFromHeight(heightMeters - ElevatorConstants.FIRST_STAGE_MAXIMUM_HEIGHT_METERS);
		}
		return getElevatorPose3dFromHeight(0);
	}

	public static Pose3d getElevatorSecondStagePosition(double heightMeters) {
		return getElevatorPose3dFromHeight(heightMeters);
	}

	private static Pose3d getElevatorPose3dFromHeight(double heightMeters) {
		return new Pose3d(new Translation3d(0, 0, heightMeters), new Rotation3d());
	}

}
