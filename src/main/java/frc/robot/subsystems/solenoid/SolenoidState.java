package frc.robot.subsystems.solenoid;

public enum SolenoidState {

	OFF(0),
	RETRACT(0.8),
	HOLD(0.2);

	private final double power;

	SolenoidState(double power) {
		this.power = power;
	}

	public double getPower() {
		return power;
	}

}
