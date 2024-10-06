package frc.robot.subsystems.roller;

public enum RollerState {

	AFTER_INTAKE(0.4),
	ROLL_IN(0.4),
	FAST_ROLL_IN(0.8),
	ROLL_OUT(-0.4),
	MANUAL(0),
	STOP(0);

	private final double power;

	RollerState(double power) {
		this.power = power;
	}

	public double getPower() {
		return power;
	}

}
