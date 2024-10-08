package frc.robot.subsystems.elevatorRoller;

public enum ElevatorRollerState {

	INTAKE_TO_ELEVATOR_ROLLER(0),
	AMP(0),
	STOP(0);

	private final double power;

	ElevatorRollerState(double power) {
		this.power = power;
	}

	public double getPower() {
		return power;
	}

}
