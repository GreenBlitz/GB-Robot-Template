package frc.robot.statemachine;

import frc.robot.Robot;
import frc.robot.statemachine.stateHandlers.StateHandler;
import frc.robot.subsystems.swerve.states.SwerveStateHandler;

public class RobotCommander extends StateHandler<RobotState> {

  private final SwerveStateHandler swerveStateHandler;

  // other SubsystemStateHandlers and StateHandlers

  public RobotCommander(String logPath, RobotState defaultState, Robot robot) {
    super(logPath, defaultState);
    this.swerveStateHandler = new SwerveStateHandler(robot.getSwerve());
  }

  @Override
  public void runByState(RobotState state) {
    switch (state) {
      default -> {}
    }
  }
}
