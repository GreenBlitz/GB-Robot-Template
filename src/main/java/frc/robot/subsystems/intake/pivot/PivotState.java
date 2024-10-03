package frc.robot.subsystems.intake.pivot;

public enum PivotState {

	DOWN(0),
	UP(1);

	private double degrees;

	PivotState(double degrees) {
		this.degrees = degrees;
	}

	public double getDegrees() {
		return degrees;
	}

}
