package frc.robot.statemachine.stateHandlers;

import java.util.Set;

public abstract class Superstructure<T> extends StateHandler<T> {

  private final Set<StateHandler> stateHandlers;

  public Superstructure(T defaultState, String logPath, StateHandler... stateHandlers) {
    super(defaultState, logPath);
    this.stateHandlers = Set.of(stateHandlers);
  }
}
