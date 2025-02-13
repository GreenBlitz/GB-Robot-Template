package frc.robot.subsystems.endeffector;

public enum EndEffectorState {

	INTAKE(0.8),
	L1_OUTTAKE(-0.4),
	BRANCH_OUTTAKE(-0.8),
	OUTTAKE(-0.8),
	DEFAULT(0.05);

	private final double power;

	EndEffectorState(double power) {
		this.power = power;
	}

	public double getPower() {
		return power;
	}

}
