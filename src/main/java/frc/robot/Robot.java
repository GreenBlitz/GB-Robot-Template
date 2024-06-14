// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.RobotConstants;
import frc.robot.simulation.MotorSimulation;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveState;
import frc.utils.CTREUtils.CANStatus;
import frc.utils.DriverStationUtils;
import frc.utils.batteryutils.Battery;
import frc.utils.cycletimeutils.CycleTimeUtils;
import frc.utils.loggerutils.LoggerUtils;
import frc.utils.pathplannerutils.PathPlannerUtils;
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
        initializeLogger();
        Battery.scheduleBatteryLimiterCommand(); //Using RobotConstants.DISABLE_BATTERY_LIMITER, disable with it!
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
        CANStatus.logAllBusStatuses();
    }

    @Override
    public void simulationPeriodic() {
        MotorSimulation.updateRegisteredSimulations();
    }

    private void initializeLogger() {
        switch (RobotConstants.ROBOT_TYPE) {
            case REAL -> {
                LoggerUtils.startRealLogger();
            }
            case SIMULATION -> {
                LoggerUtils.startSimulationLogger();
            }
            case REPLAY -> {
                setUseTiming(false); // Run as fast as possible
                LoggerUtils.startReplayLogger();
            }
        }
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
