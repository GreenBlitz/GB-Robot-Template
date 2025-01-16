package frc.robot.subsystems.endEffector;


public enum EndEffectorState {

	IDLE(0),
	INTAKE(0.5),
	OUTTAKE(-0.5);

	private final double power;

	EndEffectorState(double power) {
		this.power = power;
	}

	public double getPower() {
		return power;
	}

}
