package frc.robot.subsystems.test;

import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class TestSubsystem extends GBSubsystem<TestSubsystemState> {

  public TestSubsystem(String logPath) {
    super(logPath);
  }

  @Override
  protected void runByState(TestSubsystemState state) {
    if (state != null) {
      switch (state) {
        case STATE1 -> do1();
        case STATE2 -> do2();
        case STATE3 -> do3();
      }
    }
  }

  private void do1() {
    Logger.recordOutput(getLogPath() + "/running", "1");
  }

  private void do2() {
    Logger.recordOutput(getLogPath() + "/running", "2");
  }

  private void do3() {
    Logger.recordOutput(getLogPath() + "/running", "3");
  }
}
