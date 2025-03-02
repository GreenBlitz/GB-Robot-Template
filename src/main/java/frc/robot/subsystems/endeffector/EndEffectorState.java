package frc.robot.subsystems.endeffector;

public enum EndEffectorState {

	STOP(0),
	CORAL_INTAKE(0.9),
	ALGAE_INTAKE(-0.9),
	L1_OUTTAKE(-0.6),
	BRANCH_OUTTAKE(-0.8),
	CORAL_OUTTAKE(-0.9),
	DEFAULT(0.1),
	ALGAE_OUTTAKE(0.7),
	NET_OUTTAKE(0.8),
	PROCESSOR_OUTTAKE(0.7);

	private final double power;

	EndEffectorState(double power) {
		this.power = power;
	}

	public double getPower() {
		return power;
	}

}
