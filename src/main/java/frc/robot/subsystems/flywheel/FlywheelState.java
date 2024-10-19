package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;

public enum FlywheelState {

	DEFAULT(Rotation2d.fromRotations(20), 0.7),
	PASSING(Rotation2d.fromRotations(55), 0.7),
	PRE_SPEAKER(Rotation2d.fromRotations(60), 0.7);

	private final Rotation2d velocity;
	private final double differentialRatio;

	FlywheelState(Rotation2d velocity, double differentialRatio) {
		this.velocity = velocity;
		this.differentialRatio = differentialRatio;
	}

	public Rotation2d getRightVelocity() {
		return velocity;
	}

	public Rotation2d getLeftVelocity() {
		return velocity.times(differentialRatio);
	}

}
