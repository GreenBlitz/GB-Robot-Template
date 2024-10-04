package frc.robot.subsystems.funnel;

public enum FunnelState {

	NOTE_TO_SHOOTER(0.1),
	SPEAKER(0.2),
	SHOOTER_TO_ELEVATOR(-0.1),
	AMP(-0.2),
	ELEVATOR_TO_SHOOTER(0.3),
	STOP(0);

	private final double power;

	FunnelState(double power) {
		this.power = power;
	}

	public double getPower() {
		return power;
	}

}
