package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;

public enum FlywheelState {

	DEFAULT(Rotation2d.fromRotations(0)),
	SHOOTER_OUTTAKE(Rotation2d.fromRotations(20)),
	SHOOTING(Rotation2d.fromRotations(60));

	private final Rotation2d velocity;

	FlywheelState(Rotation2d velocity) {
		this.velocity = velocity;
	}

	public Rotation2d getVelocity() {
		return velocity;
	}

}
