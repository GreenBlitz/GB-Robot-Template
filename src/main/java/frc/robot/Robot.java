// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.superstructers.poseestimator.PoseEstimatorSuperstructure;
import frc.utils.RobotTypeUtils;
import frc.utils.auto.AutonomousChooser;
import frc.utils.auto.PathPlannerUtils;

import java.util.Optional;


public class Robot {

    public static final RobotTypeUtils.RobotType ROBOT_TYPE = RobotTypeUtils.determineRobotType(RobotTypeUtils.RobotType.SIMULATION);


    public static final Swerve swerve = new Swerve();
    public static final PoseEstimatorSuperstructure poseEstimator = new PoseEstimatorSuperstructure(swerve);
    public static AutonomousChooser autonomousChooser;

    public Robot() {
        buildPathPlannerForAuto();
        configureBindings();
    }

    private void initializeSubsystems() {

    }

    private void configureCommands() {

    }

    private void buildPathPlannerForAuto() {
        // Register commands...
        swerve.buildPathPlannerForAuto(poseEstimator::getCurrentPose, poseEstimator::resetPose);
        autonomousChooser = new AutonomousChooser("Autonomous Chooser");
        PathPlannerUtils.setTargetRotationOverride(
                () -> {
                    if (poseEstimator.getCurrentPose().getX() > 3)
                        return Optional.of(Rotation2d.fromDegrees(45));
                    else
                        return Optional.empty();
                }
        );
    }

    private void configureBindings() {
        JoysticksBindings.configureBindings();
    }

    public Command getAutonomousCommand() {
        return autonomousChooser.getChosenValue();
    }

}
