package frc.robot.statemachine.stateHandlers;

import frc.robot.subsystems.GBSubsystem;

public abstract class SubsystemStateHandler<T> extends StateHandler<T> {

  private final GBSubsystem subsystem;

  public SubsystemStateHandler(T defaultState, String logPath, GBSubsystem subsystem) {
    super(defaultState, logPath);
    this.subsystem = subsystem;
  }
}
