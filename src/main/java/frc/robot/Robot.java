// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.RobotConstants;
import frc.robot.simulation.SimulationManager;
import frc.utils.batteryutils.BatteryUtils;
import frc.utils.ctreutils.BusStatus;
import frc.utils.cycletimeutils.CycleTimeUtils;
import frc.utils.loggerutils.LoggerFactory;
import org.littletonrobotics.junction.LoggedRobot;

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
        if (RobotConstants.ROBOT_TYPE.isReplay()) {
            setUseTiming(false); // run as fast as possible
        }
        LoggerFactory.initializeLogger();
        BatteryUtils.scheduleLimiter(); // Using RobotConstants.BATTERY_LIMITER_ENABLE, disable with it!

        robotContainer = new RobotContainer();
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

    @Override
    public void robotPeriodic() {
        CycleTimeUtils.updateCycleTime(); // Better to be first
        CommandScheduler.getInstance().run();
        BusStatus.logChainsStatuses();
        BatteryUtils.logStatus();
    }

    @Override
    public void simulationPeriodic() {
        SimulationManager.updateRegisteredSimulations();
    }

}
