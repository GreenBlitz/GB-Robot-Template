package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public abstract class GBSubsystem extends SubsystemBase {

	private final String logPath;
	private Command currentCommand;

	public GBSubsystem(String logPath) {
		this.logPath = logPath;
		this.currentCommand = new InstantCommand().withName("No Command");
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
		Logger.recordOutput(getLogPath() + "CurrentCommand", getCurrentCommand().getName());
		subsystemPeriodic();
	}

	protected void subsystemPeriodic() {}

	protected Command withSetCurrentCommand(Command command){
		return new SequentialCommandGroup(
				new InstantCommand(() -> currentCommand = command),
				command
		);
	}
	
	//EXAMPLE
	public Command setPower(double power){
		return withSetCurrentCommand(
				new InstantCommand(() -> Logger.recordOutput("setpower")).withName("Set power to " + power)
		);
	}

}
