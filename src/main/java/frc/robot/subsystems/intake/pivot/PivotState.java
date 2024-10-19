package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.geometry.Rotation2d;

public enum PivotState {

	ON_FLOOR(Rotation2d.fromDegrees(4)),
	UP(Rotation2d.fromDegrees(98));

	private final Rotation2d position;

	PivotState(Rotation2d position) {
		this.position = position;
	}

	public Rotation2d getPosition() {
		return position;
	}

}
