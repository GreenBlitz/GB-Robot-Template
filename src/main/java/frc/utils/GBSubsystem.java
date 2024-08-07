package frc.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public abstract class GBSubsystem extends SubsystemBase {

	private final String logPath;

	public GBSubsystem(String logPath) {
		this.logPath = "Subsystems/" + logPath;
	}

	@Override
	public final void periodic() {
		Logger.recordOutput(getLogPath() + "Current Command", getCurrentCommandName());
		subsystemPeriodic();
	}

	private String getCurrentCommandName() {
		Command currentCommand = getCurrentCommand();
		return currentCommand != null ? currentCommand.getName() : "no command is currently running on the subsystem";
	}

	public String getLogPath() {
		return logPath;
	}

	protected abstract void subsystemPeriodic();

}
