package frc.robot;

import frc.RobotManager;

public enum RobotType {

	REAL,
	REPLAY,
	SIMULATION;

	public boolean isReal() {
		return this.equals(RobotType.REAL);
	}

	public boolean isReplay() {
		return this.equals(RobotType.REPLAY);
	}

	public boolean isSimulation() {
		return this.equals(RobotType.SIMULATION);
	}

	public static RobotType determineRobotType(boolean isReplay) {
		return isReplay ? RobotType.REPLAY : RobotManager.isSimulation() ? RobotType.SIMULATION : RobotType.REAL;
	}

}

