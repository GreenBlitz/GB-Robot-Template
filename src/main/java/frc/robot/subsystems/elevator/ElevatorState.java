package frc.robot.subsystems.elevator;

public enum ElevatorState {

	CLOSED(0),
	INTAKE(0),
	OUTTAKE(0),
	PRE_L1(0),
	PRE_L2(0.05),
	PRE_L3(0.39),
	PRE_L4(1.1),
	L1(0),
	L2(0.05),
	L3(0.39),
	L4(1.1);

	private final double heightMeters;

	ElevatorState(double heightMeters) {
		this.heightMeters = heightMeters;
	}

	public double getHeightMeters() {
		return heightMeters;
	}

}
