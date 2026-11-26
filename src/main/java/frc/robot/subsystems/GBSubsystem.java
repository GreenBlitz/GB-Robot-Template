package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.*;

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

  @Override
  public final void periodic() {
    Logger.recordOutput(getLogPath() + "/CurrentCommand", getCurrentCommand().getName());
    subsystemPeriodic();
  }

  public String getLogPath() {
    return logPath;
  }

  public Command asSubsystemCommand(Command command, String commandName) {
    command.setName(commandName);
    command.addRequirements(this);
    return command.beforeStarting(new InstantCommand(() -> currentCommand = command));
  }

  protected void subsystemPeriodic() {}
}
