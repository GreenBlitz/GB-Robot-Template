package frc.robot.subsystems.intake;


public enum IntakeState {

	INTAKE(0.3),
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
