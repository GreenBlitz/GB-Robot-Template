package frc.robot.subsystems.lifter;

public enum LifterState {

	HOLD(0, 0),
	FORWARD(0, 0.2),
	BACKWARD(0, -0.2),
	RETRACTED(0.05, -0.9),
	EXTENDED(0.49, 0.9);

	private final double targetPosition;
	private final double power;

	LifterState(double targetPosition, double power) {
		this.targetPosition = targetPosition;
		this.power = power;
	}

	public double getTargetPosition() {
		return targetPosition;
	}

	public double getPower() {
		return power;
	}

}
