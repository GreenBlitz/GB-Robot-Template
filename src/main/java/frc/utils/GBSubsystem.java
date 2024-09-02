package frc.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public abstract class GBSubsystem extends SubsystemBase {

	private final String logPath;

	public GBSubsystem(String logPath) {
		this.logPath = "Subsystems/" + logPath;
	}

	private String getCurrentCommandName() {
		Command currentCommand = getCurrentCommand();
		return currentCommand != null ? currentCommand.getName() : "no command is currently running on the subsystem";
	}

	public String getLogPath() {
		return logPath;
	}

	/**
	 * DON'T CALL THIS FUNCTION
	 */
	@Override
	@Deprecated
	public final void periodic() {}

	public final void wrapperPeriodic() {
		Logger.recordOutput(getLogPath() + "CurrentCommand", getCurrentCommandName());
		subsystemPeriodic();
	}

	protected abstract void subsystemPeriodic();

}
