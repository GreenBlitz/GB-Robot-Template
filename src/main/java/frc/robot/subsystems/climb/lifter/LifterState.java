package frc.robot.subsystems.climb.lifter;

import edu.wpi.first.math.geometry.Rotation2d;

public enum LifterState {

	HOLD(Rotation2d.fromDegrees(0), 0),
	FORWARD(Rotation2d.fromDegrees(0), 0.2),
	BACKWARD(Rotation2d.fromDegrees(0), -0.2),
	CLIMB(Rotation2d.fromDegrees(0), -0.9),
	DEPLOY(Rotation2d.fromDegrees(100), 0.4);

	private final Rotation2d targetPosition;
	private final double power;

	LifterState(Rotation2d targetPosition, double power) {
		this.targetPosition = targetPosition;
		this.power = power;
	}

	public Rotation2d getTargetPosition() {
		return targetPosition;
	}

	public double getPower() {
		return power;
	}

}
