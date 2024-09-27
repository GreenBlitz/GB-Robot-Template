package frc.robot.subsystems.funnel;

public enum FunnelState {

	INTAKE(0.3),
	OUTTAKE(-0.3),
	RELEASE_FOR_ARM(-0.3),
	STOP(0);

	private final double power;

	FunnelState(double power) {
		this.power = power;
	}

	public double getPower() {
		return power;
	}

}
