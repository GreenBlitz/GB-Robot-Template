package frc.robot.subsystems.intake.roller;

public enum IntakeStates {

	NOTE_TO_SHOOTER(0),
	INTAKE(0),
	STOP(0),
	ELEVATOR_TO_SHOOTER(0);

	private final double power;

	IntakeStates(double power) {
		this.power = power;
	}

	public double getPower() {
		return power;
	}

}
