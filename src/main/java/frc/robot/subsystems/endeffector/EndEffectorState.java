package frc.robot.subsystems.endeffector;

public enum EndEffectorState {

	INTAKE(0.4),
	OUTTAKE(-0.4),
	KEEP(0.05);

	private final double power;

	EndEffectorState(double power) {
		this.power = power;
	}

	public double getPower() {
		return power;
	}

}
