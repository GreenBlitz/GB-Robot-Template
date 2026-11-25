package frc.robot.statemachine;

import frc.robot.Robot;
import frc.robot.statemachine.superstructure.Superstructure;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.test.TestSubsystem;
import frc.robot.subsystems.test.TestSubsystemState;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.*;

public class RobotCommander extends GBSubsystem<RobotState> {

  private final Robot robot;
  private final Superstructure superstructure;
  private final PositionTargets positionTargets;

  private final TestSubsystem testSubsystem1 = new TestSubsystem("testSubsystem1");
  private final TestSubsystem testSubsystem2 = new TestSubsystem("testSubsystem2");
  private int a = 0;

  public RobotCommander(String logPath, Robot robot) {
    super(logPath);
    this.robot = robot;
    this.positionTargets = new PositionTargets(robot);
    this.superstructure = new Superstructure("StateMachine/Superstructure", robot);
  }

  @Override
  protected void runByState(RobotState state) {
    switch (state) {
      case STATE1 -> do1();
      case STATE2 -> do2();
      case STATE3 -> do3();
      case STATE4 -> do4();
    }
  }

  private void do1() {
    testSubsystem1.setState(TestSubsystemState.STATE1);
    Logger.recordOutput(getLogPath() + "/running", "1");
  }

  private void do2() {
    testSubsystem1.setState(TestSubsystemState.STATE2);
    testSubsystem2.setState(TestSubsystemState.STATE2);
    Logger.recordOutput(getLogPath() + "/running", "2");
  }

  private void do3() {
    testSubsystem1.setState(TestSubsystemState.STATE3);
    testSubsystem2.setState(TestSubsystemState.STATE1);
    Logger.recordOutput(getLogPath() + "/running", "3");
  }

  private void do4() {
    testSubsystem1.setState(TestSubsystemState.STATE1);
    Logger.recordOutput(getLogPath() + "/running", "4");
  }

  public Superstructure getSuperstructure() {
    return superstructure;
  }

  @Override
  protected void subsystemPeriodic() {
    superstructure.log();
    if (a == 0) {
      setState(RobotState.STATE1);
    }
    if (a == 500) {
      setState(RobotState.STATE2);
    }
    if (a == 1000) {
      setState(RobotState.STATE3);
    }
    if (a == 1500) {
      setState(RobotState.STATE1);
    }
    if (a == 2000) {
      setState(RobotState.STATE4);
    }
    a++;
  }

  private Command endState(RobotState state) {
    return switch (state) {
      default -> new InstantCommand();
    };
  }
}
