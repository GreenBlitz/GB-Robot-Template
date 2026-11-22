package frc.robot.subsystems;

public class TestSubsystem extends GBSubsystem {

  public TestSubsystem(String logPath) {
    super(logPath);
  }

  @Override
  protected void runByState(SubsystemState state) {
    switch (state) {
      case STATE1 -> {}
      case STATE2 -> {}
      case STATE3 -> {}
    }
  }
}
