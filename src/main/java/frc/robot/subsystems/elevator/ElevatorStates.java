package frc.robot.subsystems.elevator;

public enum ElevatorStates {

	PRE_AMP(0.1),
	AMP(0.3),
	IDLE(0.05);

	private final double position;

	ElevatorStates(double position) {
		this.position = position;
	}

	public double getPositionMeters() {
		return position;
	}

}
