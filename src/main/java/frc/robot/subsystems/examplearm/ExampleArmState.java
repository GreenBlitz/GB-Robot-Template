package frc.robot.subsystems.examplearm;

import edu.wpi.first.math.geometry.Rotation2d;

public enum ExampleArmState {

	INTAKE(Rotation2d.fromDegrees(-75)),
	LOW_DROP(Rotation2d.fromDegrees(-10)),
	MID_DROP(Rotation2d.fromDegrees(50)),
	HIGH_DROP(Rotation2d.fromDegrees(80)),
	SAFE_HOLD(Rotation2d.fromDegrees(-80)),
	STAY_IN_PLACE(null);


	private final Rotation2d position;

	ExampleArmState(Rotation2d position) {
		this.position = position;
	}

	public Rotation2d getPosition() {
		return position;
	}

}
