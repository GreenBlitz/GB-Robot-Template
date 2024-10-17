package frc.robot.subsystems.funnel;

public enum FunnelState {

	SPEAKER(0.8),
	AMP(-0.6),
	INTAKE_OUTTAKE(-0.8),
	SHOOTER_OUTTAKE(0.7),
	NOTE_TO_SHOOTER(0.5),
	SHOOTER_TO_ELEVATOR(-0.7),
	STOP(0);

	private final double power;

	FunnelState(double power) {
		this.power = power;
	}

	public double getPower() {
		return power;
	}

}
