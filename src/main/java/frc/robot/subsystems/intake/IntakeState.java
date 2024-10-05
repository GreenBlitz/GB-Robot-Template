package frc.robot.subsystems.intake;

public enum IntakeState {

	INTAKE(0.8),
	INTAKE_WITH_FUNNEL(0.4),
	INTAKE_WITH_ARM(0.4),
	RELEASE_FOR_ARM(0.4),
	OUTTAKE(-0.3),
	STOP(0);

	private final double power;

	IntakeState(double power) {
		this.power = power;
	}

	public double getPower() {
		return power;
	}

}
