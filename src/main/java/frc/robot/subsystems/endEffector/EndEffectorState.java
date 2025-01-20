package frc.robot.subsystems.endEffector;

public enum EndEffectorState {

	INTAKE(0.4),
	OUTTAKE(-0.4),
	IDLE(0.1);

	double power;

	EndEffectorState(double power) {
		this.power = power;
	}

	public double getPower() {
		return power;
	}

}
