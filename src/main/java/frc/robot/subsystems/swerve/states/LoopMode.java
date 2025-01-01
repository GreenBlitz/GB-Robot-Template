package frc.robot.subsystems.swerve.states;

public enum LoopMode {

	CLOSED(true),
	OPEN(false);

	private final boolean isClosedLoop;

	LoopMode(boolean isClosedLoop) {
		this.isClosedLoop = isClosedLoop;
	}

	public boolean isClosedLoop() {
		return isClosedLoop;
	}

}
