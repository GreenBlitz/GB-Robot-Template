package frc.robot.subsystems.funnel;

public enum FunnelState {

	SPEAKER(0.6),
	AMP(-0.6),
	INTAKE_OUTTAKE(-0.4),
	SHOOTER_OUTTAKE(0.4),
	NOTE_TO_SHOOTER(0.5),
	SHOOTER_TO_ELEVATOR(-0.5),
	STOP(0);

	private final double power;

	FunnelState(double power) {
		this.power = power;
	}

	public double getPower() {
		return power;
	}

}
