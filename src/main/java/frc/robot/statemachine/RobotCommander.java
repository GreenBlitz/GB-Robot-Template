package frc.robot.statemachine;

import frc.robot.Robot;
import frc.robot.statemachine.stateHandlers.SubsystemStateHandler;
import frc.robot.statemachine.stateHandlers.Superstructure;

public class RobotCommander extends Superstructure<RobotState> {

  private final SubsystemStateHandler swerveStateHandler;

  public RobotCommander(RobotState defaultState, String logPath, Robot robot) {
    super(defaultState, logPath, robot.getSwerve().getStateHandler());
    this.swerveStateHandler = robot.getSwerve().getStateHandler();
  }

  @Override
  public void runByState(RobotState state) {
    switch (state) {
      default -> {}
    }
  }
}
