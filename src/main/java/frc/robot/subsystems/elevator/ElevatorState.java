package frc.robot.subsystems.elevator;

public enum ElevatorState {

	CLOSED(0.02),
	INTAKE(0.02),
	OUTTAKE(0.02),
	PRE_L1(0),
	L1(0.02),
	PRE_L2(0.02),
	L2(0.05),
	PRE_L3(0.32),
	L3(0.32),
	WHILE_DRIVE_L4(0.35),
	PRE_L4(1.18),
	L4(1.18);

	private final double heightMeters;

	ElevatorState(double heightMeters) {
		this.heightMeters = heightMeters;
	}

	public double getHeightMeters() {
		return heightMeters;
	}

}
