// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.RobotConstants;
import frc.robot.simulation.MotorSimulation;
import frc.utils.loggerutils.LoggerUtils;
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
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        initializeLogger();
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
        CommandScheduler.getInstance().run();
        MotorSimulation.updateRegisteredSimulations();
    }

    private void initializeLogger() {
        NetworkTableInstance.getDefault().getStructTopic("RobotPose", Pose2d.struct).publish();
        NetworkTableInstance.getDefault().getStructTopic("MechanismPoses", Pose3d.struct).publish();

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
}
