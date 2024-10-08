package frc.robot.subsystems.funnel;

public enum FunnelState {

	INTAKE(0.55),
	SLOW_INTAKE(0.2),
	SHOOT(0.7),
	OUTTAKE(-0.5),
	SLOW_OUTTAKE(-0.2),
	RELEASE_FOR_ARM(-0.3),
	TRANSFER_TO_ARM(-0.7),
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
