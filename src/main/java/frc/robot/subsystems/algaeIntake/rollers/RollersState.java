package frc.robot.subsystems.algaeIntake.rollers;

public enum RollersState {

	IDLE(0.1),
	INTAKE(0.5),
	TRANSFER_TO_END_EFFECTOR(-0.5),
	OUTTAKE(-0.5);

	private final double power;

	RollersState(double power) {
		this.power = power;
	}

	public double getPower() {
		return power;
	}

}
