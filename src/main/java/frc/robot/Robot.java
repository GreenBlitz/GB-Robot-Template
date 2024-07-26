// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.swervecontainer.SwerveFactory;
import frc.robot.subsystems.swerve.swervestatehelpers.SwerveStateHandler;
import frc.robot.superstructers.poseestimator.PoseEstimatorSuperstructure;
import frc.utils.RobotTypeUtils;


public class Robot {

    public static final RobotTypeUtils.RobotType ROBOT_TYPE = RobotTypeUtils.determineRobotType(RobotTypeUtils.RobotType.REAL);


    public static final Swerve swerve = new Swerve(
            SwerveFactory.createConstants(),
            SwerveFactory.createModules(),
            SwerveFactory.createGyro()
    );
    public static final PoseEstimatorSuperstructure poseEstimator = new PoseEstimatorSuperstructure(swerve);
    public static final SwerveStateHandler swerveStateHandler = new SwerveStateHandler(
            poseEstimator::getCurrentPose,
            swerve
    );

    static {
        swerve.applyStateHandler(swerveStateHandler);
    }
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
    }

    private void configureBindings() {
        JoysticksBindings.configureBindings();
    }

    public Command getAutonomousCommand() {
        return new InstantCommand();
    }

}
