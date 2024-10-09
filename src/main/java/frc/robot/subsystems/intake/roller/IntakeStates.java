package frc.robot.subsystems.intake.roller;

public enum IntakeStates {

	INTAKE(0),
	NOTE_TO_SHOOTER(0),
	SHOOTER_TO_ELEVATOR(0),
	OUTTAKE(0),
	STOP(0);

	private final double power;

	IntakeStates(double power) {
		this.power = power;
	}

	public double getPower() {
		return power;
	}

}
