package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.geometry.Rotation2d;

public enum PivotState {

	DOWN(Rotation2d.fromDegrees(0)),
	UP(Rotation2d.fromDegrees(1));

	private Rotation2d positionDegrees;

	PivotState(Rotation2d positionDegrees) {
		this.positionDegrees = positionDegrees;
	}

	public double getPositionDegrees() {
		return positionDegrees.getDegrees();
	}

}
