package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.*;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

public abstract class GBSubsystem extends SubsystemBase {

	private final String logPath;
	private Command currentCommand;

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

	@Override
	public final void periodic() {
		double startingTime = TimeUtil.getCurrentTimeSeconds();

		Logger.recordOutput(getLogPath() + "/CurrentCommand", getCurrentCommand().getName());
		subsystemPeriodic();

		Logger.recordOutput("TimeTest/" + getLogPath(), TimeUtil.getCurrentTimeSeconds() - startingTime);
	}

	protected void subsystemPeriodic() {}

	public Command asSubsystemCommand(Command command, String commandName) {
		command.setName(commandName);
		command.addRequirements(this);
		return command.beforeStarting(new InstantCommand(() -> currentCommand = command));
	}

}
