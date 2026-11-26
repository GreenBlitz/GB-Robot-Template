package frc.robot.statemachine.stateHandlers;

import org.wpilib.command2.Command;
import org.wpilib.command2.RunCommand;

public abstract class StateHandler<T> {

  private final String logPath;
  private T currentState;
  private Command stateCommand;

  public StateHandler(T defaultState, String logPath) {
    this.logPath = logPath;
    this.currentState = defaultState;

    this.stateCommand = new RunCommand(() -> runByState(getCurrentState()));
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
