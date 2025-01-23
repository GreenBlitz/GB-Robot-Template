package frc.robot.subsystems.elevator;

public enum ElevatorState {

	CLOSED(0.01),
	FEEDER(0.36),
	L1(0.01),
	L2(0.1),
	L3(0.3),
	L4(0.9),
	OUTTAKE(0.3);

	private final double heightMeters;

	ElevatorState(double heightMeters) {
		this.heightMeters = heightMeters;
	}

	public double getHeightMeters() {
		return heightMeters;
	}

}
