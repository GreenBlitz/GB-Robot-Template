package frc.robot.subsystems.funnel;

public enum FunnelState {

	INTAKE(0.5),
	SHOOT(0.7),
	OUTTAKE(-0.3),
	RELEASE_FOR_ARM(-0.3),
	TRANSFER_TO_ARM(-0.7),
	TRANSFER_TO_SHOOTER(0.7),
	MANUAL(0),
	STOP(0);

	private final double power;

	FunnelState(double power) {
		this.power = power;
	}

	public double getPower() {
		return power;
	}

}
