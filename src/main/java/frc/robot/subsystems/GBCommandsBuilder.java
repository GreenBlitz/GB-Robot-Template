package frc.robot.subsystems;

public class GBCommandsBuilder {

	private boolean isSubsystemRunningIndependently;

	public GBCommandsBuilder() {
		this.isSubsystemRunningIndependently = false;
	}

	public boolean isSubsystemRunningIndependently() {
		return isSubsystemRunningIndependently;
	}

	public void setIsSubsystemRunningIndependently(boolean isSubsystemRunningIndependently) {
		this.isSubsystemRunningIndependently = isSubsystemRunningIndependently;
	}

}
