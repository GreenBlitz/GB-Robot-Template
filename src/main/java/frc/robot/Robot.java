// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveState;
import frc.utils.DriverStationUtils;
import frc.utils.pathplannerutils.PathPlannerUtils;
import frc.robot.simulation.SimulationManager;
import frc.utils.battery.BatteryUtils;
import frc.utils.ctre.BusStatus;
import frc.utils.cycletime.CycleTimeUtils;
import frc.utils.logger.LoggerFactory;
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
        PathPlannerUtils.startPathPlanner();

        robotContainer = new RobotContainer();
        buildPathPlannerForAuto(); // Must happen after robot container
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
        RobotContainer.POSE_ESTIMATOR.periodic();
        BusStatus.logChainsStatuses();
        BatteryUtils.logStatus();
    }

    @Override
    public void simulationPeriodic() {
        SimulationManager.updateRegisteredSimulations();
    }

    private void buildPathPlannerForAuto() {
        //register commands
        PathPlannerUtils.configurePathPlanner(
                RobotContainer.POSE_ESTIMATOR::getCurrentPose,
                RobotContainer.POSE_ESTIMATOR::resetPose,
                RobotContainer.SWERVE::getSelfRelativeVelocity,
                (speeds) -> RobotContainer.SWERVE.driveByState(speeds, SwerveState.DEFAULT_PATH_PLANNER),
                SwerveConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG,
                DriverStationUtils::isRedAlliance,
                RobotContainer.SWERVE
        );
    }

}
