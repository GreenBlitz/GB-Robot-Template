package frc.robot.subsystems;

public class GBCommandsBuilder {

	private boolean isRunningIndependently;

	public GBCommandsBuilder() {
		this.isRunningIndependently = false;
	}

	public boolean isRunningIndependently() {
		return isRunningIndependently;
	}

	public void setIsRunningIndependently(boolean isRunningIndependently) {
		this.isRunningIndependently = isRunningIndependently;
	}

}
