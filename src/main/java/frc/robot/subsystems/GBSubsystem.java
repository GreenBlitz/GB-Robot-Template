package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.*;
import org.littletonrobotics.junction.Logger;

public abstract class GBSubsystem extends SubsystemBase {

	private final String logPath;
	private Command currentCommand;
	private boolean isRunningIndependently;

	public GBSubsystem(String logPath) {
		this.logPath = logPath;
		this.currentCommand = Commands.none().withName("None");
	}

	@Override
	public Command getCurrentCommand() {
		return currentCommand;
	}

	public String getLogPath() {
		return logPath;
	}

	public boolean isRunningIndependently() {
		return isRunningIndependently;
	}

	public void setIsRunningIndependently(boolean isRunningIndependently) {
		this.isRunningIndependently = isRunningIndependently;
	}

	@Override
	public final void periodic() {
		Logger.recordOutput(getLogPath() + "/CurrentCommand", getCurrentCommand().getName());
		Logger.recordOutput(getLogPath() + "/IsRunningIndependently", isRunningIndependently());
		subsystemPeriodic();
	}

	protected void subsystemPeriodic() {}

	public Command asSubsystemCommand(Command command, String commandName) {
		command.setName(commandName);
		command.addRequirements(this);
		return command.beforeStarting(new InstantCommand(() -> currentCommand = command));
	}

}
