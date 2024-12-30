package frc.robot.swerve.states;

public enum LoopMode {

	CLOSED(true),
	OPEN(false);

	public final boolean isClosedLoop;

	LoopMode(boolean isClosedLoop) {
		this.isClosedLoop = isClosedLoop;
	}

}
