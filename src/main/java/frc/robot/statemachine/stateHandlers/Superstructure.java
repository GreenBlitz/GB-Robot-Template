package frc.robot.statemachine.stateHandlers;

import java.util.Set;

public abstract class Superstructure<T> extends StateHandler<T> {

  private final Set<StateHandler> stateHandlers;

  public Superstructure(T defaultState, String logPath, Set<StateHandler> stateHandlers) {
    super(logPath, defaultState);
    this.stateHandlers = stateHandlers;
  }
}
