package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;

public enum ElevatorStates {

	PRE_AMP(0),
	AMP(0),
	IDLE(0);

	private final double position;

	ElevatorStates(double position) {
		this.position = position;
	}

	public double getPositionMeters() {
		return position;
	}

}
