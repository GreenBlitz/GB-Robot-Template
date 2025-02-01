package frc.robot.subsystems.elevator;

public enum ElevatorState {

	CLOSED(0.01),
	INTAKE(0.36),
	OUTTAKE(0.3),
	L1(0.01),
	L2(0.01),
	L3(0.1),
	L4(0.8);

	private final double heightMeters;

	ElevatorState(double heightMeters) {
		this.heightMeters = heightMeters;
	}

	public double getHeightMeters() {
		return heightMeters;
	}

}
