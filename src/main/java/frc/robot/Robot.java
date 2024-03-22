// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.SimulationConstants;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;

  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    initializeLogger();
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }
  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }


  public static RobotType getRobotType() {
    RobotType robotType = RobotConstants.ROBOT_TYPE;
    if (isSimulation()) {
      if (robotType.equals(RobotType.REPLAY)) {
        return RobotType.REPLAY;
      }
      return RobotType.SIMULATION;
    } else {
      if (robotType.equals(RobotType.REAL)) {
        return RobotType.REAL;
      }
    }
    return RobotType.REAL;
  }

  private void initializeLogger() {
    NetworkTableInstance.getDefault()
            .getStructTopic("RobotPose", Pose2d.struct).publish();

    NetworkTableInstance.getDefault()
            .getStructTopic("MechanismPoses", Pose3d.struct).publish();
    switch (getRobotType()) {
      // Running on a real robot, log to a USB stick
      case REAL -> {
        try {
          Logger.addDataReceiver(new WPILOGWriter(RobotConstants.USB_LOG_PATH));
          System.out.println("initialized Logger, USB");
        } catch (Exception e) {
          Logger.end();
          Logger.addDataReceiver(new WPILOGWriter(RobotConstants.SAFE_ROBORIO_LOG_PATH));
          System.out.println("initialized Logger, roborio");
        }
        Logger.addDataReceiver(new NT4Publisher());
      }
      // Replaying a log, set up replay source
      case REPLAY -> {
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_simulation")));
      }
      default -> {
        Logger.addDataReceiver(new NT4Publisher());
        Logger.addDataReceiver(new WPILOGWriter(SimulationConstants.SIMULATION_LOG_PATH));
      }
    }
    Logger.start();
  }

  public enum RobotType {
    REAL,
    SIMULATION,
    REPLAY
  }
}
