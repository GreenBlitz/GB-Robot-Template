package frc.robot.subsystems.algaeIntake.pivot;

import edu.wpi.first.math.geometry.Rotation2d;

public enum PivotState {

	CLOSED(Rotation2d.fromDegrees(115)),
	INTAKE(Rotation2d.fromDegrees(-11)),
	TRANSFER_TO_END_EFFECTOR(Rotation2d.fromDegrees(100)),
	OUTTAKE(Rotation2d.fromDegrees(-10)),
	HOLD_ALGAE(Rotation2d.fromDegrees(20)),
	STAY_IN_PLACE(null);

	private final Rotation2d position;

	PivotState(Rotation2d position) {
		this.position = position;
	}

	public Rotation2d getPosition() {
		return position;
	}


}
