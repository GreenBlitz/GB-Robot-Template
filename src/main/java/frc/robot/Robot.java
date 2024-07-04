// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    public static final Swerve swerve = new Swerve(); // Must be before POSE_ESTIMATOR
    public static final PoseEstimator poseEstimator = new PoseEstimator(RobotConstants.DEFAULT_POSE);

    public Robot() {
        buildPathPlannerForAuto();
        configureBindings();
    }

    private void buildPathPlannerForAuto() {
        // Register commands...
        PathPlannerUtils.configurePathPlanner(
                Robot.poseEstimator::getCurrentPose,
                Robot.poseEstimator::resetPose,
                Robot.swerve::getSelfRelativeVelocity,
                (speeds) -> Robot.swerve.driveByState(speeds, SwerveState.DEFAULT_PATH_PLANNER),
                SwerveConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG,
                DriverStationUtils::isRedAlliance,
                Robot.swerve
        );
    }

    private void configureBindings() {
        JoysticksBindings.configureBindings();
    }

    public Command getAutonomousCommand() {
        return new InstantCommand();
    }


    private static boolean isAtTranslationPosition(double currentTranslationVelocity, double currentTranslationPosition, double targetTranslationPosition) {
        boolean isNearTargetPosition = MathUtil.isNear(
                targetTranslationPosition,
                currentTranslationPosition,
                RobotConstants.TRANSLATION_TOLERANCE_METERS
        );
        boolean isStopping = Math.abs(currentTranslationVelocity) < RobotConstants.TRANSLATION_VELOCITY_TOLERANCE;
        return isNearTargetPosition && isStopping;
    }

    public static boolean isAtXAxisPosition(double targetXBlueAlliancePosition) {
        return isAtTranslationPosition(
                swerve.getFieldRelativeVelocity().vxMetersPerSecond,
                poseEstimator.getCurrentPose().getX(),
                targetXBlueAlliancePosition
        );
    }

    public static boolean isAtYAxisPosition(double targetYBlueAlliancePosition) {
        return isAtTranslationPosition(
                swerve.getFieldRelativeVelocity().vyMetersPerSecond,
                poseEstimator.getCurrentPose().getY(),
                targetYBlueAlliancePosition
        );
    }

    public static boolean isAtAngle(Rotation2d targetAngle) {
        double angleDifferenceDeg = Math.abs(targetAngle.minus(poseEstimator.getCurrentPose().getRotation()).getDegrees());
        boolean isAtAngle = angleDifferenceDeg < RobotConstants.ROTATION_TOLERANCE.getDegrees();

        double currentRotationVelocityRadians = swerve.getSelfRelativeVelocity().omegaRadiansPerSecond;
        boolean isStopping = Math.abs(currentRotationVelocityRadians) < RobotConstants.ROTATION_VELOCITY_TOLERANCE.getRadians();

        return isAtAngle && isStopping;
    }

    public static boolean isAtPosition(Pose2d targetBluePose) {
        return isAtXAxisPosition(targetBluePose.getX())
                && isAtYAxisPosition(targetBluePose.getY())
                && isAtAngle(targetBluePose.getRotation()
        );
    }

    // reset pose
    // update pose
    // get pose
}
