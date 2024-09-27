package frc.robot.subsystems.roller;

public enum RollerState {

	INTAKE(0.4),
	OUTTAKE(-0.4),
	STOP(0);

	private final double power;

	RollerState(double power) {
		this.power = power;
	}

	public double getPower() {
		return power;
	}

}
