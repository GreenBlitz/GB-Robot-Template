package frc.robot;

import frc.RobotManager;

public enum RobotType {

	REAL,
	SIMULATION,
	REPLAY;

	public boolean isReal() {
		return this.equals(RobotType.REAL);
	}

	public boolean isSimulation() {
		return this.equals(RobotType.SIMULATION);
	}

	public boolean isReplay() {
		return this.equals(RobotType.REPLAY);
	}

	public static RobotType determineRobotType(boolean isReplay) {
		return RobotManager.isSimulation() ? (isReplay ? REPLAY : SIMULATION) : REAL;
	}

}

