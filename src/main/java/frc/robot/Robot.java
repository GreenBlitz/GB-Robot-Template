// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.RobotConstants;
import frc.robot.simulation.MotorSimulation;
import frc.utils.allianceutils.AllianceRotation2d;
import frc.utils.allianceutils.AllianceTranslation2d;
import frc.utils.batteryutils.Battery;
import frc.utils.loggerutils.LoggerUtils;
import frc.utils.roborioutils.RoborioUtils;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

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
        RoborioUtils.updateRioUtils(); // Better to be first
        CommandScheduler.getInstance().run();
        MotorSimulation.updateRegisteredSimulations();
        AllianceTranslation2d speaker = AllianceTranslation2d.fromBlueAllianceTranslation(0, 5);
        AllianceTranslation2d currentPose = AllianceTranslation2d.fromBlueAllianceTranslation(2, 7);
        Logger.recordOutput("Blue Speaker", speaker.getBlueAllianceTranslation2d());
        Logger.recordOutput("Red Speaker", speaker.getMirroredAllianceTranslation2d());
        Logger.recordOutput("Current Pose", currentPose.getBlueAllianceTranslation2d());
        Rotation2d wantedAngle = Rotation2d.fromRadians(
                Math.atan2(
                        speaker.getMirroredAllianceTranslation2d().getY() - currentPose.getBlueAllianceTranslation2d().getY(),
                        speaker.getMirroredAllianceTranslation2d().getX() - currentPose.getBlueAllianceTranslation2d().getX()
                )
        );
        AllianceRotation2d allianceWantedAngle = AllianceRotation2d.fromBlueAlliancePose(wantedAngle);
        Logger.recordOutput("wantedAngleDeg", allianceWantedAngle.getAllianceAngle().getDegrees());
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
