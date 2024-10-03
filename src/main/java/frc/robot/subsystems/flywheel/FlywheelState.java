package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;

public enum FlywheelState {

	IDLE(Rotation2d.fromRotations(40)),
	SHOOTING(Rotation2d.fromRotations(60));

	Rotation2d velocity;

	FlywheelState(Rotation2d velocity) {
		this.velocity = velocity;
	}

	public Rotation2d getPower() {
		return velocity;
	}

}
