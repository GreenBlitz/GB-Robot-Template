package frc.robot.statemachine.intakestatehandler;

import edu.wpi.first.math.geometry.Rotation2d;

public enum IntakeState {

	CLOSED(Rotation2d.fromDegrees(50), 0),
	INTAKE(Rotation2d.fromDegrees(15), 0.8),
	CALIBRATION(Rotation2d.fromDegrees(15), 0.8),
	STAY_IN_PLACE(Rotation2d.fromRotations(Double.NaN), Double.NaN);

	private final Rotation2d fourBarPosition;
	private final double intakePower;

	IntakeState(Rotation2d fourBarPosition, double intakePower) {
		this.fourBarPosition = fourBarPosition;
		this.intakePower = intakePower;
	}

	public Rotation2d getFourBarPosition() {
		return fourBarPosition;
	}

	public double getIntakePower() {
		return intakePower;
	}

}
