// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.simulation.SimulationManager;
import frc.utils.AlertLogger;
import frc.utils.battery.BatteryUtils;
import frc.utils.ctre.BusStatus;
import frc.utils.cycletime.CycleTimeUtils;
import frc.utils.logger.LoggerFactory;
import frc.utils.pathplannerutils.PathPlannerUtils;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;


public class RobotManager extends LoggedRobot {

    private Command autonomousCommand;

    private Robot robot;


    @Override
    public void robotInit() {
        if (Robot.ROBOT_TYPE.isReplay()) {
            setUseTiming(false); // run as fast as possible
        }
        LoggerFactory.initializeLogger();
        BatteryUtils.scheduleLimiter(); // Using RobotConstants.BATTERY_LIMITER_ENABLE, disable with it!
        PathPlannerUtils.startPathPlanner();
        PathPlannerUtils.setLoggingPathToPaths((pose) -> Logger.recordOutput("Current Path To " + "Follow", pose.toArray(new Pose2d[0])));

        this.robot = new Robot();
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robot.getAutonomousCommand();

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

    @Override
    public void robotPeriodic() {
        CycleTimeUtils.updateCycleTime(); // Better to be first
        Robot.poseEstimator.updatePoseEstimator();
        CommandScheduler.getInstance().run();
        BusStatus.logChainsStatuses();
        BatteryUtils.logStatus();
    }

    @Override
    public void simulationPeriodic() {
        SimulationManager.updateRegisteredSimulations();
    }

}
