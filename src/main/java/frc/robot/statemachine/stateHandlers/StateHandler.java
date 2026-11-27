package frc.robot.statemachine.stateHandlers;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.RunCommand;

public abstract class StateHandler<T> {

  protected Command stateCommand;
  private final String logPath;
  private T currentState;

  public StateHandler(String logPath, T defaultState) {
    this.stateCommand = new RunCommand(() -> runByState(getCurrentState()));
    this.logPath = logPath;
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

  public void log() {
    Logger.recordOutput(getLogPath() + "/currentState", getCurrentState().toString());
  }
}
