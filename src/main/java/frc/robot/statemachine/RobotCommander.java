package frc.robot.statemachine;

import frc.robot.Robot;
import frc.robot.statemachine.stateHandlers.Superstructure;

public class RobotCommander extends Superstructure<RobotState> {

  public RobotCommander(RobotState defaultState, String logPath, Robot robot) {
    super(defaultState, logPath, robot.getStateHandlers());
  }
}
