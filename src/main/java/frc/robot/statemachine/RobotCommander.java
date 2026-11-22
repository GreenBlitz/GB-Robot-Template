package frc.robot.statemachine;

import frc.robot.Robot;
import frc.robot.statemachine.superstructure.Superstructure;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.TestSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import org.wpilib.command2.*;

public class RobotCommander extends GBSubsystem {

  private final Robot robot;
  private final Swerve swerve;
  private final Superstructure superstructure;
  private final PositionTargets positionTargets;

  private final TestSubsystem testSubsystem;
  private final TestSubsystem testSubsystem2;

  private SubsystemState currentState;

  public RobotCommander(String logPath, Robot robot) {
    super(logPath);
    this.robot = robot;
    //        this.swerve = robot.getSwerve();
    this.swerve = null;
    this.positionTargets = new PositionTargets(robot);
    this.superstructure = new Superstructure("StateMachine/Superstructure", robot);
    this.currentState = null;

    this.testSubsystem = new TestSubsystem("TestSubsystem");
    this.testSubsystem2 = new TestSubsystem("TestSubsystem2");
  }

  public SubsystemState getCurrentState() {
    return currentState;
  }

  public void setStateA() {
    testSubsystem.setState(SubsystemState.STATE1);
  }

  public void setStateB() {
    testSubsystem.setState(SubsystemState.STATE2);
    testSubsystem2.setState(SubsystemState.STATE1);
  }

  public void setStateC() {
    testSubsystem.setState(SubsystemState.STATE3);
    testSubsystem2.setState(SubsystemState.STATE2);
  }

  public Superstructure getSuperstructure() {
    return superstructure;
  }

  @Override
  protected void subsystemPeriodic() {
    superstructure.log();
  }

  private Command endState(RobotState state) {
    return switch (state) {
      default -> new InstantCommand();
    };
  }
}
