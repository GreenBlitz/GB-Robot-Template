package frc.robot.subsystems;

import frc.utils.utilcommands.InitExecuteCommand;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.*;

public abstract class GBSubsystem<T> extends SubsystemBase {

  private final String logPath;
  private Command currentCommand;
  private InitExecuteCommand stateCommand;
  private T currentState;

  public GBSubsystem(String logPath) {
    this.logPath = logPath;
    this.currentCommand = Commands.none().withName("None");
    this.stateCommand = new InitExecuteCommand(() -> {}, () -> runByState(getCurrentState()));
    CommandScheduler.getInstance().schedule(asSubsystemCommand(stateCommand, "stateCommand"));
  }

  public void setState(T state) {
    this.currentState = state;
  }

  public T getCurrentState() {
    return currentState;
  }

  protected void runByState(T state) {}

  @Override
  public Command getCurrentCommand() {
    return currentCommand;
  }

  public String getLogPath() {
    return logPath;
  }

  @Override
  public final void periodic() {
    Logger.recordOutput(getLogPath() + "/CurrentCommand", getCurrentCommand().getName());
    //    Logger.recordOutput(getLogPath() + "/CurrentState", getCurrentState().name());
    subsystemPeriodic();
  }

  protected void subsystemPeriodic() {}

  public Command asSubsystemCommand(Command command, String commandName) {
    command.setName(commandName);
    command.addRequirements(this);
    return command.beforeStarting(new InstantCommand(() -> currentCommand = command));
  }
}
