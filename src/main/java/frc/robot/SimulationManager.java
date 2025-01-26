package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.elevator.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

public class SimulationManager {

	private final String logPath;
	private final Robot robot;

	private final static double ELEVATOR_STAGE_HEIGHT = 0.93;

	public SimulationManager(String logPath, Robot robot) {
		this.logPath = logPath;
		this.robot = robot;
	}

	public void logPoses() {
		logElevatorPosition3d();
		logArmPosition3d();
	}

	private void logElevatorPosition3d() {
		Logger.recordOutput(
			logPath + "/Elevator/FirstStagePosition",
			getElevatorFirstStagePosition(robot.getElevator().getElevatorPositionMeters())
		);
		Logger.recordOutput(
			logPath + "/Elevator/SecondStagePosition",
			getElevatorSecondStagePosition(robot.getElevator().getElevatorPositionMeters())
		);
	}

	private void logArmPosition3d() {
		Logger.recordOutput(
			logPath + "/Arm/Position",
			getArmPose3dFromHeight(
				robot.getElevator().getElevatorPositionMeters() + ELEVATOR_STAGE_HEIGHT,
				-robot.getArm().getPosition().getRadians()
			)
		);
	}

	private static Pose3d getElevatorFirstStagePosition(double heightMeters) {
		if (heightMeters > ElevatorConstants.FIRST_STAGE_MAXIMUM_HEIGHT_METERS) {
			return getElevatorPose3dFromHeight(heightMeters - ElevatorConstants.FIRST_STAGE_MAXIMUM_HEIGHT_METERS);
		}
		return getElevatorPose3dFromHeight(0);
	}

	private static Pose3d getElevatorSecondStagePosition(double heightMeters) {
		return getElevatorPose3dFromHeight(heightMeters);
	}

	private static Pose3d getElevatorPose3dFromHeight(double heightMeters) {
		return new Pose3d(new Translation3d(0, 0, heightMeters), new Rotation3d());
	}

	private static Pose3d getArmPose3dFromHeight(double heightMeters, double pitch) {
		return new Pose3d(new Translation3d(0, 0, heightMeters), new Rotation3d(0.0, pitch, 0.0));
	}

}
