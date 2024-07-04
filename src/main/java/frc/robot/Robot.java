// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.poseestimation.poseestimator.PoseEstimator;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveState;
import frc.utils.DriverStationUtils;
import frc.utils.pathplannerutils.PathPlannerUtils;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link RobotManager}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

    public static final Swerve SWERVE = new Swerve(); // Must be before POSE_ESTIMATOR

    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator();

    public Robot() {
        buildPathPlannerForAuto();
        configureBindings();
    }

    private void buildPathPlannerForAuto() {
        // Register commands...
        PathPlannerUtils.configurePathPlanner(
                Robot.POSE_ESTIMATOR::getCurrentPose,
                Robot.POSE_ESTIMATOR::resetPose,
                Robot.SWERVE::getSelfRelativeVelocity,
                (speeds) -> Robot.SWERVE.driveByState(speeds, SwerveState.DEFAULT_PATH_PLANNER),
                SwerveConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG,
                DriverStationUtils::isRedAlliance,
                Robot.SWERVE
        );
    }

    private void configureBindings() {
        JoysticksBindings.configureBindings();
    }

    public Command getAutonomousCommand() {
        return new InstantCommand();
    }

}
