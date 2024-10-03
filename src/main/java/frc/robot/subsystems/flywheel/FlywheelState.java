package frc.robot.subsystems.flywheel;

public enum FlywheelState {

	IDLE(0.8),
	SHOOTING(0.8);

	double power;

	FlywheelState(double power) {
		this.power = power;
	}

	public double getPower() {
		return power;
	}

}
