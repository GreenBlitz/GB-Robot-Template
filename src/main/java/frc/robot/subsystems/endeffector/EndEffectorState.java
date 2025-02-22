package frc.robot.subsystems.endeffector;

public enum EndEffectorState {

	CORAL_INTAKE(0.7),
	ALGAE_INTAKE(-0.9),
	L1_OUTTAKE(-0.4),
	BRANCH_OUTTAKE(-0.8),
	CORAL_OUTTAKE(-0.7),
	DEFAULT(0.1),
<<<<<<< HEAD
	PROCESSOR_OUTTAKE(0.4),
	NET_OUTTAKE(0.7);
=======
	ALGAE_OUTTAKE(0.5);
>>>>>>> master

	private final double power;

	EndEffectorState(double power) {
		this.power = power;
	}

	public double getPower() {
		return power;
	}

}
