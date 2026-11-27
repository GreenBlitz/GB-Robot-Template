package frc.robot.statemachine.stateHandlers;

import org.wpilib.command2.Command;
import org.wpilib.command2.RunCommand;

public abstract class StateHandler<T> {

  private final String logPath;
  protected Command stateCommand;
  private T currentState;

  public StateHandler(String logPath, T defaultState) {
    this.logPath = logPath;
    this.stateCommand = new RunCommand(() -> runByState(getCurrentState()));
    this.currentState = defaultState;
  }

  public String getLogPath() {
    return logPath;
  }

  public T getCurrentState() {
    return currentState;
  }

  public void setState(T state) {
    currentState = state;
  }

  public void runByState(T state) {}

  public void log() {}
}
