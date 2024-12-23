package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public abstract class GBSubsystem extends SubsystemBase {

	private final String logPath;
	private String currentCommandName;

	public GBSubsystem(String logPath) {
		this.logPath = logPath;
		this.currentCommandName = "No command is currently running on the"
	}
	
	public String getLogPath() {
		return logPath;
	}

	@Override
	public final void periodic() {
		Logger.recordOutput(getLogPath() + "CurrentCommand", currentCommandName);
		subsystemPeriodic();
	}

	protected void subsystemPeriodic() {}

	protected Command addNameToCommand(Command command){
		return new SequentialCommandGroup(
				new InstantCommand(() -> currentCommandName = command.getName()),
				command
		);
	}
	
	//EXAMPLE
	public Command setPower(double power){
		return addNameToCommand(
				new InstantCommand(() -> Logger.recordOutput("setpower")).withName("Set power to " + power)
		);
	}

}
