package frc.robot.subsystems.climb.solenoid;

public enum SolenoidState {

	LOCKED(0),
	INITIAL_FREE(0.8),
	HOLD_FREE(0.2);

	private final double power;

	SolenoidState(double power) {
		this.power = power;
	}

	public double getPower() {
		return power;
	}

}
