package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public abstract class GBSubsystem extends SubsystemBase {

	private final Supplier<String> currentCommandNameSupplier;
	private final String logPath;

	public GBSubsystem(String logPath, Supplier<String> currentCommandNameSupplier) {
		this.currentCommandNameSupplier = currentCommandNameSupplier;
		this.logPath = logPath;
	}

	public String getLogPath() {
		return logPath;
	}

	@Override
	public final void periodic() {
		Logger.recordOutput(getLogPath() + "CurrentCommand", currentCommandNameSupplier.get());
		subsystemPeriodic();
	}

	protected void subsystemPeriodic() {}

}
