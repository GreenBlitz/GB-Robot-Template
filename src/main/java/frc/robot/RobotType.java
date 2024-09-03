package frc.robot;

public enum RobotType {

	REAL,
	SIMULATION;

	public boolean isReal() {
		return this.equals(RobotType.REAL);
	}

	public boolean isSimulation() {
		return this.equals(RobotType.SIMULATION);
	}

	public static RobotType determineRobotType() {
		return RobotManager.isSimulation() ? RobotType.SIMULATION : RobotType.REAL;
	}

}

