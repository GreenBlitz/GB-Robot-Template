package frc.robot.subsystems.swerve.swervestatehelpers;

public enum LoopMode {

	CLOSED(true),
	OPEN(false);

	public final boolean isClosedLoop;

	LoopMode(boolean isClosedLoop) {
		this.isClosedLoop = isClosedLoop;
	}

}
