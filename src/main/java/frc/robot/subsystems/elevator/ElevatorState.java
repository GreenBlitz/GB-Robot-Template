package frc.robot.subsystems.elevator;

public enum ElevatorState {

	CLOSED(0.05),
	FEEDER(1),
	L1(0.5),
	L2(0.85),
	L3(1.25),
	L4(1.90),
	OUTTAKE(0.3);

	private final double heightMeters;

	ElevatorState(double heightMeters) {
		this.heightMeters = heightMeters;
	}

	public double getHeightMeters() {
		return heightMeters;
	}

}
