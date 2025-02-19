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
	L4(1.18),
	LOW_ALGAE_REMOVE(2),
	HIGH_ALGAE_REMOVE(0.25),
	NET_WHILE_DRIVE(2),
	PRE_NET(2),
	NET(2);


	private final double heightMeters;

	ElevatorState(double heightMeters) {
		this.heightMeters = heightMeters;
	}

	public double getHeightMeters() {
		return heightMeters;
	}

}
