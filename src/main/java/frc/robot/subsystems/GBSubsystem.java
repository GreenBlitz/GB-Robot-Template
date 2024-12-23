package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public abstract class GBSubsystem extends SubsystemBase {

	private final String logPath;

	public GBSubsystem(String logPath) {
		this.logPath = logPath;
	}

	public String getLogPath() {
		return logPath;
	}

	@Override
	public final void periodic() {
		Logger.recordOutput(getLogPath() + "CurrentCommand", getCurrentCommandName());
		subsystemPeriodic();
	}

	protected void subsystemPeriodic() {}

	public abstract String getCurrentCommandName();

}
