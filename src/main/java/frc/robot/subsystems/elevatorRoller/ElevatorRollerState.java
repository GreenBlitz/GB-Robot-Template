package frc.robot.subsystems.elevatorRoller;

public enum ElevatorRollerState {

	TRANSFER_FROM_ELEVATOR(-0.4),
	TRANSFER_TO_ELEVATOR(0.4),
	AMP(0.5),
	STOP(0);

	private final double power;

	ElevatorRollerState(double power) {
		this.power = power;
	}

	public double getPower() {
		return power;
	}

}
