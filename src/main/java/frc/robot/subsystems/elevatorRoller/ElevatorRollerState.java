package frc.robot.subsystems.elevatorRoller;

public enum ElevatorRollerState {

	TRANSFER_FROM_ELEVATOR(0),
	TRANSFER_TO_ELEVATOR(0),
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
