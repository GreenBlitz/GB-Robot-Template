package frc.robot.subsystems.endEffector;

public enum EndEffectorState {

	INTAKE(0.2),
	OUTTAKE(-0.2),
	IDLE(0);

	double power;

	EndEffectorState(double power) {
		this.power = power;
	}

	public double getPower() {
		return power;
	}

}
