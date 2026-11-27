package frc.robot.statemachine.stateHandlers;

import frc.robot.subsystems.GBSubsystem;

public abstract class SubsystemStateHandler<T> extends StateHandler<T> {

  private final GBSubsystem subsystem;

  public SubsystemStateHandler(T defaultState, String logPath, GBSubsystem subsystem) {
    super(logPath, defaultState);
    this.subsystem = subsystem;
    super.stateCommand =
        subsystem.asSubsystemCommand(super.stateCommand, super.stateCommand.getName());
  }
}
