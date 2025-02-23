package frc.robot.subsystems.elevator;

public enum ElevatorState {

	STAY_IN_PLACE(Double.NaN),
	CLOSED(0.02),
	INTAKE(0.05),
	ALGAE_OUTTAKE(0.02),
	PRE_L1(0),
	L1(0.02),
	PRE_L2(0.02),
	L2(0.02),
	PRE_L3(0.16),
	L3(0.16),
	WHILE_DRIVE_L4(0.35),
	PRE_L4(1.18),
	L4(1.18),
	LOW_ALGAE_REMOVE(0.02),
	POST_LOW_ALGAE_REMOVE(0.02),
	HIGH_ALGAE_REMOVE(0.4),
	POST_HIGH_ALGAE_REMOVE(0.4),
	WHILE_DRIVE_NET(0.4),
	NET(1.18),
	PROCESSOR_OUTTAKE(0.02);


	private final double heightMeters;

	ElevatorState(double heightMeters) {
		this.heightMeters = heightMeters;
	}

	public double getHeightMeters() {
		return heightMeters;
	}

}
