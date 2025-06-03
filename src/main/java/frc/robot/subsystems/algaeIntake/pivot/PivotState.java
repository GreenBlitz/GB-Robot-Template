package frc.robot.subsystems.algaeIntake.pivot;

import edu.wpi.first.math.geometry.Rotation2d;

public enum PivotState {

	CLOSED(Rotation2d.fromDegrees(110)),
	INTAKE(Rotation2d.fromDegrees(0)),
	TRANSFER_TO_END_EFFECTOR(Rotation2d.fromDegrees(100)),
	OUTTAKE(Rotation2d.fromDegrees(0));

	private final Rotation2d position;

	PivotState(Rotation2d position) {
		this.position = position;
	}

	public Rotation2d getPosition() {
		return position;
	}

}
