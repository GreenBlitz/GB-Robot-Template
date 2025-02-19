package frc.robot.subsystems.endeffector;

public enum EndEffectorState {

	CORAL_INTAKE(0.7),
	ALGAE_INTAKE(0.7),
	L1_OUTTAKE(-0.4),
	BRANCH_OUTTAKE(-0.8),
	OUTTAKE(-0.7),
	DEFAULT(0.05);

	private final double power;

	EndEffectorState(double power) {
		this.power = power;
	}

	public double getPower() {
		return power;
	}

}
